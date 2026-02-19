#!/usr/bin/env python3
"""
Camera Tilt Servo Controller — MAVROS DO_SET_ACTUATOR interface.

Controls a peripheral actuator (servo) connected to the flight controller
to tilt the camera between three preset positions:

  forward  — default, camera faces straight ahead
  half     — camera tilted halfway down
  down     — camera faces straight down

PX4 "Peripheral Actuator Set 1" outputs are controlled via
MAV_CMD_DO_SET_ACTUATOR (187) with normalized values (-1.0 to 1.0).
The normalized value maps to PWM through the output's min/max config
in PX4:  -1 → PWM_MIN,  0 → center,  +1 → PWM_MAX.

Interfaces (all three can be used simultaneously):

  RC switch (set rc_channel parameter to enable):
    3-position switch → forward / half / down
    Thresholds: < 1250 µs = forward, > 1750 µs = down, between = half

  Topic (Foxglove Publish panel):
    ~/command  (std_msgs/String)  — publish "forward", "half", or "down"

  Services (programmatic):
    ~/set_forward   (std_srvs/Trigger)
    ~/set_half      (std_srvs/Trigger)
    ~/set_down      (std_srvs/Trigger)

  Feedback:
    ~/state    (std_msgs/String)  — publishes current state after each change

Usage from CLI:
  ros2 topic pub --once /camera_tilt/command std_msgs/msg/String "{data: forward}"
  ros2 topic pub --once /camera_tilt/command std_msgs/msg/String "{data: half}"
  ros2 topic pub --once /camera_tilt/command std_msgs/msg/String "{data: down}"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import RCIn

# MAVLink command IDs
MAV_CMD_DO_SET_ACTUATOR = 187


class CameraTiltController(Node):
    """ROS2 node that drives a camera-tilt servo via MAVROS."""

    STATES = ("forward", "half", "down")

    def __init__(self):
        super().__init__("camera_tilt")

        # ── Parameters ────────────────────────────────────────────────
        # actuator_index: which actuator within the set (1-6 → param1-param6)
        self.declare_parameter("actuator_index", 1)
        # Normalized values: -1.0 to 1.0
        # Maps to PWM via PX4 output config: -1 → PWM_MIN, 0 → center, +1 → PWM_MAX
        self.declare_parameter("value_forward", -1.0)
        self.declare_parameter("value_half", 0.0)
        self.declare_parameter("value_down", 1.0)
        self.declare_parameter("mavros_cmd_service",
                               "/mavros/cmd/command")
        # RC input: map a 3-position switch to tilt states
        # Set to 0 to disable RC control
        self.declare_parameter("rc_channel", 0)
        # Thresholds for 3-position switch (PWM µs from /mavros/rc/in)
        self.declare_parameter("rc_threshold_low", 1250)
        self.declare_parameter("rc_threshold_high", 1750)

        self._actuator_index = self.get_parameter("actuator_index").value
        self._values = {
            "forward": self.get_parameter("value_forward").value,
            "half":    self.get_parameter("value_half").value,
            "down":    self.get_parameter("value_down").value,
        }
        mavros_srv = self.get_parameter("mavros_cmd_service").value
        self._rc_channel = self.get_parameter("rc_channel").value
        self._rc_thresh_low = self.get_parameter("rc_threshold_low").value
        self._rc_thresh_high = self.get_parameter("rc_threshold_high").value

        # Validate actuator index
        if not 1 <= self._actuator_index <= 6:
            self.get_logger().fatal(
                f"actuator_index must be 1-6, got {self._actuator_index}"
            )
            raise SystemExit(1)

        # ── MAVROS CommandLong client ─────────────────────────────────
        self._cmd_client = self.create_client(CommandLong, mavros_srv)
        self.get_logger().info(
            f"Waiting for MAVROS command service: {mavros_srv} ..."
        )
        if not self._cmd_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                f"MAVROS service {mavros_srv} not available yet — "
                "commands will be retried when called."
            )

        # ── Trigger services ──────────────────────────────────────────
        self._srv_forward = self.create_service(
            Trigger, "~/set_forward", self._cb_forward
        )
        self._srv_half = self.create_service(
            Trigger, "~/set_half", self._cb_half
        )
        self._srv_down = self.create_service(
            Trigger, "~/set_down", self._cb_down
        )

        # ── String topic interface (for Foxglove) ─────────────────────
        # Subscribe: publish "forward", "half", or "down" to ~/command
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._cmd_sub = self.create_subscription(
            String, "~/command", self._cb_command, reliable_qos
        )
        # Publish: current state after each change on ~/state
        self._state_pub = self.create_publisher(
            String, "~/state", reliable_qos
        )

        # ── RC input (optional, 3-position switch) ───────────────────
        if self._rc_channel > 0:
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self._rc_sub = self.create_subscription(
                RCIn, "/mavros/rc/in", self._cb_rc_in, sensor_qos
            )
            self._rc_last_state = None
            self.get_logger().info(
                f"RC control enabled on channel {self._rc_channel} "
                f"(thresholds: <{self._rc_thresh_low}=forward, "
                f">{self._rc_thresh_high}=down)"
            )
        else:
            self._rc_sub = None
            self.get_logger().info("RC control disabled (rc_channel=0)")

        self._current_state = None

        # ── Set initial position (forward) ────────────────────────────
        self.get_logger().info(
            f"Camera tilt controller ready  "
            f"(actuator={self._actuator_index}, "
            f"fwd={self._values['forward']}, "
            f"half={self._values['half']}, "
            f"down={self._values['down']})"
        )
        # Send initial servo position after a short delay so MAVROS is ready
        self._init_timer = self.create_timer(2.0, self._set_initial_position)

    # ── Initial position ──────────────────────────────────────────────

    def _set_initial_position(self):
        """Set servo to forward on startup, then cancel the one-shot timer."""
        self._init_timer.cancel()
        self._set_tilt("forward")

    # ── RC callback ────────────────────────────────────────────────

    def _cb_rc_in(self, msg: RCIn):
        """Map a 3-position RC switch to tilt states."""
        idx = self._rc_channel - 1  # 1-indexed → 0-indexed
        if idx >= len(msg.channels):
            return

        pwm = msg.channels[idx]
        if pwm == 0 or pwm == 65535:
            return  # no data / failsafe

        if pwm < self._rc_thresh_low:
            state = "forward"
        elif pwm > self._rc_thresh_high:
            state = "down"
        else:
            state = "half"

        # Only send command on state change to avoid spamming
        if state != self._rc_last_state:
            self._rc_last_state = state
            self._set_tilt(state)

    # ── Topic callback (Foxglove-friendly) ───────────────────────────

    def _cb_command(self, msg: String):
        """Handle ~/command topic: accepts 'forward', 'half', or 'down'."""
        state = msg.data.strip().lower()
        if state not in self.STATES:
            self.get_logger().warn(
                f"Unknown command '{msg.data}' — "
                f"use one of: {', '.join(self.STATES)}"
            )
            return
        self._set_tilt(state)

    # ── Service callbacks ─────────────────────────────────────────────

    def _cb_forward(self, _request, response):
        return self._handle_tilt("forward", response)

    def _cb_half(self, _request, response):
        return self._handle_tilt("half", response)

    def _cb_down(self, _request, response):
        return self._handle_tilt("down", response)

    def _handle_tilt(self, state: str, response):
        """Service handler: send actuator command and populate Trigger response."""
        sent = self._set_tilt(state)
        if sent:
            response.success = True
            response.message = (
                f"Camera tilt → '{state}' "
                f"(actuator={self._actuator_index}, "
                f"value={self._values[state]})"
            )
        else:
            response.success = False
            response.message = "MAVROS command service not available"
        return response

    # ── Common tilt logic ─────────────────────────────────────────────

    def _set_tilt(self, state: str) -> bool:
        """Send actuator command (async) and publish state feedback."""
        sent = self._send_actuator_command(state)
        if sent:
            self._current_state = state
            # Publish current state for Foxglove / subscribers
            state_msg = String()
            state_msg.data = state
            self._state_pub.publish(state_msg)
        return sent

    # ── MAVROS actuator command ───────────────────────────────────────

    def _send_actuator_command(self, state: str) -> bool:
        """
        Send MAV_CMD_DO_SET_ACTUATOR via MAVROS CommandLong (async).

        param1-param6 correspond to actuators 1-6 in the peripheral set.
        Unused actuators are set to NaN (no change).
        param7 = 0 for Actuator Set 1.

        Returns True if the request was sent, False if the service is
        unavailable.  The actual PX4 result is logged asynchronously.
        """
        if not self._cmd_client.service_is_ready():
            self.get_logger().warn("MAVROS command service not ready")
            return False

        value = float(self._values[state])
        NAN = float("nan")  # NaN = "no change" for unused actuators

        # Build param1-param6: set only the target actuator, NaN for the rest
        params = [NAN] * 6
        params[self._actuator_index - 1] = value  # 1-indexed → 0-indexed

        req = CommandLong.Request()
        req.broadcast = False
        req.command = MAV_CMD_DO_SET_ACTUATOR
        req.confirmation = 0
        req.param1 = params[0]
        req.param2 = params[1]
        req.param3 = params[2]
        req.param4 = params[3]
        req.param5 = params[4]
        req.param6 = params[5]
        req.param7 = 0.0    # Index 0 = Actuator Set 1

        future = self._cmd_client.call_async(req)
        future.add_done_callback(
            lambda f, s=state, v=value: self._on_actuator_response(f, s, v)
        )

        self.get_logger().info(
            f"Camera tilt → '{state}' "
            f"(actuator={self._actuator_index}, value={value})"
        )
        return True

    def _on_actuator_response(self, future, state: str, value: float):
        """Callback for async MAVROS CommandLong response."""
        try:
            result = future.result()
            if result is not None and result.success:
                self.get_logger().debug(
                    f"DO_SET_ACTUATOR OK: actuator={self._actuator_index} "
                    f"value={value}"
                )
            else:
                rc = result.result if result else "no response"
                self.get_logger().error(
                    f"DO_SET_ACTUATOR REJECTED: actuator={self._actuator_index} "
                    f"value={value} rc={rc}"
                )
        except Exception as e:
            self.get_logger().error(f"DO_SET_ACTUATOR exception: {e}")


# ---------------------------------------------------------------------------
#  Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CameraTiltController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
