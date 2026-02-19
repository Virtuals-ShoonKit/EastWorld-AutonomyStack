#!/usr/bin/env python3
"""
Standalone CSI camera test node — run isolated without the full stack.

Tests the GStreamer pipeline, encoder, and ROS2 publishing in isolation.
Includes a --test-pattern mode using videotestsrc (no real camera needed).

Usage:
  # Real CSI camera (default)
  ros2 run eastworld_camera test_camera

  # Synthetic test pattern (no camera required)
  ros2 run eastworld_camera test_camera --ros-args -p source:=test

  # JPEG mode instead of H.264
  ros2 run eastworld_camera test_camera --ros-args -p encoding:=jpeg

  # Custom resolution / bitrate
  ros2 run eastworld_camera test_camera --ros-args -p width:=640 -p height:=480 -p h264_bitrate:=1000000

  # Then in another terminal, verify the topic:
  ros2 topic hz /test_camera/image/h264
  ros2 topic bw /test_camera/image/h264

  # Or connect Foxglove Studio (run bridge first):
  ros2 run foxglove_bridge foxglove_bridge
"""

import subprocess
import sys
import time

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402


# ---------------------------------------------------------------------------
#  Diagnostics — run before the node starts
# ---------------------------------------------------------------------------
def _check_gst_element(name: str) -> bool:
    """Return True if a GStreamer element factory exists."""
    return Gst.ElementFactory.find(name) is not None


def _run_diagnostics(logger):
    """Print system / GStreamer diagnostics to help debug issues."""
    logger.info("=" * 60)
    logger.info("  CSI Camera — Standalone Test")
    logger.info("=" * 60)

    # GStreamer version
    ver = Gst.version()
    logger.info(f"GStreamer version: {ver[0]}.{ver[1]}.{ver[2]}.{ver[3]}")

    # Check critical elements
    elements = {
        "nvarguscamerasrc": "CSI camera source (Argus)",
        "nvv4l2h264enc": "HW H.264 encoder (NVENC)",
        "nvv4l2h265enc": "HW H.265 encoder (NVENC)",
        "nvvidconv": "NVMM ↔ system memory converter",
        "h264parse": "H.264 bitstream parser",
        "jpegenc": "Software JPEG encoder",
        "videotestsrc": "Test pattern generator",
        "appsink": "Application sink",
    }

    logger.info("GStreamer elements:")
    all_ok = True
    for elem, desc in elements.items():
        found = _check_gst_element(elem)
        status = "OK" if found else "MISSING"
        if not found:
            all_ok = False
        logger.info(f"  {elem:25s} {status:8s}  ({desc})")

    if not all_ok:
        logger.warn(
            "Some GStreamer elements are missing. "
            "Install nvidia-l4t-gstreamer or gstreamer1.0-plugins-*"
        )

    # Check /dev/video* devices (V4L2 — USB cameras, not CSI)
    try:
        result = subprocess.run(
            ["ls", "-la", "/dev/video*"],
            capture_output=True,
            text=True,
            timeout=2,
        )
        if result.returncode == 0:
            logger.info("V4L2 devices:")
            for line in result.stdout.strip().split("\n"):
                logger.info(f"  {line.strip()}")
        else:
            logger.info("No /dev/video* (V4L2) devices — normal for CSI cameras")
            logger.info("  CSI cameras use nvargus (Argus API), not V4L2")
    except Exception:
        pass

    # Check nvargus-daemon
    try:
        result = subprocess.run(
            ["pgrep", "-a", "nvargus-daemon"],
            capture_output=True,
            text=True,
            timeout=2,
        )
        if result.returncode == 0:
            logger.info(f"nvargus-daemon: RUNNING (pid {result.stdout.strip().split()[0]})")
        else:
            logger.warn(
                "nvargus-daemon: NOT RUNNING — "
                "start with: sudo systemctl start nvargus-daemon"
            )
    except Exception:
        pass

    logger.info("-" * 60)


# ---------------------------------------------------------------------------
#  Lazy msg imports
# ---------------------------------------------------------------------------
_CompressedVideo = None
_CompressedImage = None


def _get_compressed_video():
    global _CompressedVideo
    if _CompressedVideo is None:
        try:
            from foxglove_msgs.msg import CompressedVideo
        except ImportError:
            print(
                "\n[FATAL] foxglove_msgs not installed.\n"
                "  Install with:  sudo apt install ros-humble-foxglove-msgs\n"
                "  Or use JPEG mode instead:  --ros-args -p encoding:=jpeg\n"
            )
            sys.exit(1)
        _CompressedVideo = CompressedVideo
    return _CompressedVideo


def _get_compressed_image():
    global _CompressedImage
    if _CompressedImage is None:
        from sensor_msgs.msg import CompressedImage
        _CompressedImage = CompressedImage
    return _CompressedImage


# ---------------------------------------------------------------------------
#  Test node
# ---------------------------------------------------------------------------
class TestCamera(Node):
    """Standalone camera test — real CSI or synthetic test pattern."""

    def __init__(self):
        super().__init__("test_camera")

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("source", "csi")  # "csi" or "test"
        self.declare_parameter("sensor_id", 0)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("fps", 30)
        self.declare_parameter("encoding", "h264")  # "h264" or "jpeg"
        self.declare_parameter("h264_bitrate", 2_000_000)
        self.declare_parameter("h264_gop", 30)
        self.declare_parameter("jpeg_quality", 50)
        self.declare_parameter("frame_id", "csi_camera")
        self.declare_parameter("topic", "/test_camera/image")
        self.declare_parameter("duration", 0)  # 0 = run forever, >0 = seconds then exit

        g = self.get_parameter
        self._source = g("source").value
        self._sensor_id = g("sensor_id").value
        self._width = g("width").value
        self._height = g("height").value
        self._fps = g("fps").value
        self._encoding = g("encoding").value
        self._bitrate = g("h264_bitrate").value
        self._gop = g("h264_gop").value
        self._jpeg_q = g("jpeg_quality").value
        self._frame_id = g("frame_id").value
        topic_base = g("topic").value
        self._duration = g("duration").value

        # ── Diagnostics ───────────────────────────────────────────────
        Gst.init(None)
        _run_diagnostics(self.get_logger())

        self.get_logger().info(f"Source:   {self._source}")
        self.get_logger().info(f"Encoding: {self._encoding}")
        self.get_logger().info(f"Resolution: {self._width}x{self._height}@{self._fps}")
        if self._encoding == "h264":
            self.get_logger().info(f"Bitrate: {self._bitrate / 1e6:.1f} Mbps, GOP: {self._gop}")
        else:
            self.get_logger().info(f"JPEG quality: {self._jpeg_q}")

        # ── Validate source ───────────────────────────────────────────
        if self._source == "csi" and not _check_gst_element("nvarguscamerasrc"):
            self.get_logger().error(
                "nvarguscamerasrc not available. "
                "Use source:=test for test pattern, or install NVIDIA GStreamer plugins."
            )
            sys.exit(1)

        if self._source == "csi" and self._encoding == "h264" and not _check_gst_element("nvv4l2h264enc"):
            self.get_logger().error(
                "nvv4l2h264enc not available. "
                "Use encoding:=jpeg, or install NVIDIA GStreamer plugins."
            )
            sys.exit(1)

        # ── Publisher ─────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        if self._encoding == "h264":
            MsgType = _get_compressed_video()
            self._topic = topic_base + "/h264"
        else:
            MsgType = _get_compressed_image()
            self._topic = topic_base + "/compressed"

        self._pub = self.create_publisher(MsgType, self._topic, qos)
        self.get_logger().info(f"Publishing on: {self._topic}")

        # ── GStreamer pipeline ────────────────────────────────────────
        pipe_str = self._build_pipeline()
        self.get_logger().info(f"Pipeline:\n  {pipe_str}")

        try:
            self._pipeline = Gst.parse_launch(pipe_str)
        except Exception as e:
            self.get_logger().fatal(f"Pipeline creation failed: {e}")
            sys.exit(1)

        self._sink = self._pipeline.get_by_name("sink")
        self._sink.set_property("emit-signals", True)
        self._sink.connect("new-sample", self._on_sample)

        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_bus_error)
        bus.connect("message::eos", self._on_bus_eos)

        ret = self._pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().fatal(
                "Pipeline FAILED to start.\n"
                + (
                    "  Check CSI camera connection and nvargus-daemon."
                    if self._source == "csi"
                    else "  videotestsrc should always work — check GStreamer install."
                )
            )
            sys.exit(1)

        self.get_logger().info("=" * 60)
        self.get_logger().info("  STREAMING — waiting for frames...")
        self.get_logger().info(f"  Verify with: ros2 topic hz {self._topic}")
        self.get_logger().info(f"               ros2 topic bw {self._topic}")
        self.get_logger().info("=" * 60)

        # Stats
        self._n_frames = 0
        self._total_bytes = 0
        self._t_start = time.monotonic()
        self._t_stats = time.monotonic()
        self._t_first_frame = None

        # Duration timer
        if self._duration > 0:
            self.create_timer(float(self._duration), self._on_duration_timeout)

    # ── Pipeline construction ─────────────────────────────────────────

    def _build_pipeline(self) -> str:
        if self._source == "test":
            # Synthetic SMPTE test pattern — works without any camera
            src = (
                f"videotestsrc pattern=smpte is-live=true ! "
                f"video/x-raw,"
                f"width={self._width},height={self._height},"
                f"framerate={self._fps}/1,format=I420"
            )

            if self._encoding == "h264":
                # SW x264enc for test pattern (no NVMM)
                enc = (
                    f"x264enc tune=zerolatency bitrate={self._bitrate // 1000} "
                    f"key-int-max={self._gop} speed-preset=ultrafast "
                    f"! h264parse "
                    f"! video/x-h264,stream-format=byte-stream,alignment=au"
                )
            else:
                enc = f"jpegenc quality={self._jpeg_q}"
        else:
            # Real CSI camera via nvarguscamerasrc
            src = (
                f"nvarguscamerasrc sensor-id={self._sensor_id} ! "
                f"video/x-raw(memory:NVMM),"
                f"width={self._width},height={self._height},"
                f"framerate={self._fps}/1,format=NV12"
            )

            if self._encoding == "h264":
                enc = (
                    f"nvv4l2h264enc "
                    f"bitrate={self._bitrate} "
                    f"preset-level=1 "
                    f"insert-sps-pps=true "
                    f"iframeinterval={self._gop} "
                    f"maxperf-enable=true "
                    f"! h264parse "
                    f"! video/x-h264,stream-format=byte-stream,alignment=au"
                )
            else:
                enc = (
                    f"nvvidconv ! video/x-raw,format=I420 ! "
                    f"jpegenc quality={self._jpeg_q}"
                )

        sink = "appsink name=sink sync=false max-buffers=2 drop=true"
        return f"{src} ! {enc} ! {sink}"

    # ── GStreamer callbacks ────────────────────────────────────────────

    def _on_sample(self, appsink):
        sample = appsink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR

        data = bytes(info.data)
        buf.unmap(info)

        # First frame latency
        if self._t_first_frame is None:
            self._t_first_frame = time.monotonic()
            startup_ms = (self._t_first_frame - self._t_start) * 1000
            self.get_logger().info(f"First frame received! (startup latency: {startup_ms:.0f} ms)")

        now = self.get_clock().now().to_msg()

        if self._encoding == "h264":
            msg = _get_compressed_video()()
            msg.timestamp = now
            msg.frame_id = self._frame_id
            msg.format = "h264"
            msg.data = data
        else:
            msg = _get_compressed_image()()
            msg.header.stamp = now
            msg.header.frame_id = self._frame_id
            msg.format = "jpeg"
            msg.data = data

        self._pub.publish(msg)

        # Stats
        self._n_frames += 1
        self._total_bytes += len(data)
        t = time.monotonic()
        dt = t - self._t_stats
        if dt >= 3.0:
            fps = self._n_frames / dt
            kb = len(data) / 1024
            avg_kb = (self._total_bytes / max(self._n_frames, 1)) / 1024
            mbps = avg_kb * 8 * fps / 1024
            self.get_logger().info(
                f"  {fps:.1f} fps | {kb:.0f} KB/frame (avg {avg_kb:.0f}) | ~{mbps:.1f} Mbps"
            )
            self._n_frames = 0
            self._total_bytes = 0
            self._t_stats = t

        return Gst.FlowReturn.OK

    def _on_bus_error(self, _bus, msg):
        err, debug = msg.parse_error()
        self.get_logger().error(f"GStreamer error: {err.message}")
        if debug:
            self.get_logger().error(f"  debug info: {debug}")

    def _on_bus_eos(self, _bus, _msg):
        self.get_logger().info("End of stream")

    def _on_duration_timeout(self):
        elapsed = time.monotonic() - self._t_start
        self.get_logger().info(f"Test duration ({self._duration}s) reached. Shutting down.")
        self.get_logger().info(f"Total runtime: {elapsed:.1f}s")
        raise SystemExit(0)

    # ── Cleanup ───────────────────────────────────────────────────────

    def destroy_node(self):
        if hasattr(self, "_pipeline"):
            self._pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


# ---------------------------------------------------------------------------
#  Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TestCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
