#!/usr/bin/env python3
"""
CSI Camera Streamer for Jetson — HW-accelerated H.264/JPEG to Foxglove.

Captures a MIPI-CSI camera via nvarguscamerasrc, encodes with Jetson
NVENC hardware, and publishes compressed frames for Foxglove Bridge
to relay over WiFi with minimal latency and bandwidth.

  H.264 mode (default):  ~2 Mbps at 720p30 — ideal for WiFi
    publishes foxglove_msgs/CompressedVideo on <topic>/h264

  JPEG mode (fallback):  ~5-15 Mbps at 720p30
    publishes sensor_msgs/CompressedImage on <topic>/compressed

Architecture:
  nvarguscamerasrc ─► NVMM ─► nvv4l2h264enc ─► ROS2 topic
                                                     │
                                               Foxglove Bridge
                                                     │
                                              WiFi (WebSocket)
                                                     │
                                              Foxglove Studio
"""

import sys
import time

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

# ---------------------------------------------------------------------------
#  Lazy-imported message types (avoid hard failure if foxglove_msgs missing)
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
                "  Or use JPEG mode instead:  -p encoding:=jpeg\n"
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
#  Node
# ---------------------------------------------------------------------------
class CSIStreamer(Node):
    """Publishes HW-compressed camera frames from a Jetson CSI camera."""

    def __init__(self):
        super().__init__("csi_streamer")

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("sensor_id", 0)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("fps", 30)
        self.declare_parameter("encoding", "h264")  # "h264" or "jpeg"
        self.declare_parameter("h264_bitrate", 2_000_000)  # bps
        self.declare_parameter("h264_gop", 30)  # keyframe every N frames
        self.declare_parameter("jpeg_quality", 50)  # 1-100 (software jpegenc)
        self.declare_parameter("frame_id", "csi_camera")
        self.declare_parameter("topic", "/csi_camera/image")

        g = self.get_parameter
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

        # ── Publisher (best-effort, depth-1 for lowest latency) ───────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        if self._encoding == "h264":
            MsgType = _get_compressed_video()
            self._topic = topic_base + "/h264"
            self._pub = self.create_publisher(MsgType, self._topic, qos)
            self.get_logger().info(
                f"H.264 → {self._topic}  "
                f"({self._width}x{self._height}@{self._fps}, "
                f"{self._bitrate / 1e6:.1f} Mbps)"
            )
        elif self._encoding == "jpeg":
            MsgType = _get_compressed_image()
            self._topic = topic_base + "/compressed"
            self._pub = self.create_publisher(MsgType, self._topic, qos)
            self.get_logger().info(
                f"JPEG → {self._topic}  "
                f"({self._width}x{self._height}@{self._fps}, q={self._jpeg_q})"
            )
        else:
            self.get_logger().fatal(
                f"Unknown encoding '{self._encoding}' — use 'h264' or 'jpeg'"
            )
            sys.exit(1)

        # ── GStreamer pipeline ────────────────────────────────────────
        Gst.init(None)
        pipe_str = self._build_pipeline()
        self.get_logger().info(f"GStreamer pipeline:\n  {pipe_str}")

        try:
            self._pipeline = Gst.parse_launch(pipe_str)
        except Exception as e:
            self.get_logger().fatal(f"Failed to create GStreamer pipeline: {e}")
            sys.exit(1)

        # Connect appsink
        self._sink = self._pipeline.get_by_name("sink")
        self._sink.set_property("emit-signals", True)
        self._sink.connect("new-sample", self._on_sample)

        # Monitor bus for errors
        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_bus_error)

        ret = self._pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().fatal(
                "Pipeline failed to start. Check:\n"
                "  - CSI camera connected?  (ls /dev/video*)\n"
                "  - nvargus-daemon running?  (sudo systemctl start nvargus-daemon)\n"
                "  - Correct sensor_id?"
            )
            sys.exit(1)

        self.get_logger().info("Streaming ✓")

        # Stats
        self._n_frames = 0
        self._t_stats = time.monotonic()

    # ── Pipeline construction ─────────────────────────────────────────

    def _build_pipeline(self) -> str:
        src = (
            f"nvarguscamerasrc sensor-id={self._sensor_id} ! "
            f"video/x-raw(memory:NVMM),"
            f"width={self._width},height={self._height},"
            f"framerate={self._fps}/1,format=NV12"
        )

        if self._encoding == "h264":
            # Hardware H.264 encoder — stays entirely in NVMM (zero-copy)
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
            # JPEG: copy NVMM → system memory via nvvidconv, then SW jpegenc
            # (nvjpegenc is not reliably available on all JP6 builds)
            enc = (
                f"nvvidconv ! video/x-raw,format=I420 ! "
                f"jpegenc quality={self._jpeg_q}"
            )

        sink = "appsink name=sink sync=false max-buffers=2 drop=true"
        return f"{src} ! {enc} ! {sink}"

    # ── GStreamer callbacks (called from streaming thread) ─────────────

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

        # Periodic stats
        self._n_frames += 1
        t = time.monotonic()
        dt = t - self._t_stats
        if dt >= 5.0:
            fps = self._n_frames / dt
            kb = len(data) / 1024
            mbps = kb * 8 * fps / 1024
            self.get_logger().info(
                f"{fps:.1f} fps | {kb:.0f} KB/frame | ~{mbps:.1f} Mbps"
            )
            self._n_frames = 0
            self._t_stats = t

        return Gst.FlowReturn.OK

    def _on_bus_error(self, _bus, msg):
        err, debug = msg.parse_error()
        self.get_logger().error(f"GStreamer error: {err.message}")
        if debug:
            self.get_logger().debug(f"  debug: {debug}")

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
    node = CSIStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
