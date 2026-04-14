"""
Stub node for rotary / indexed motors.
Simulates behavior so the full ROS2 graph can be tested without hardware.
Replace with the real driver node when the motor + controller arrive.

Subscribed topics (relative to namespace):
  ~/motor_spin   [std_msgs/Float32]  — set target RPM (rotary motors)
  ~/motor_stop   [std_msgs/Empty]    — stop rotation
  ~/motor_step   [std_msgs/Empty]    — advance one position (conveyor)

Published topics:
  ~/motor_status [std_msgs/String]   — "idle" | "spinning" | "stepping" | "error"

Parameters:
  motor_id          (str)   — label for log messages
  spin_up_time_s    (float) — delay before publishing "spinning" (default 0.3)
  spin_down_time_s  (float) — delay before publishing "idle" after stop (default 0.2)
  step_time_s       (float) — conveyor step duration before publishing "idle" (default 1.5)
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Empty, String


class RotaryMotorStubNode(Node):

    def __init__(self):
        super().__init__("rotary_motor_stub")

        self.declare_parameter("motor_id",         "motor")
        self.declare_parameter("spin_up_time_s",   0.3)
        self.declare_parameter("spin_down_time_s", 0.2)
        self.declare_parameter("step_time_s",      1.5)

        self._motor_id       = self.get_parameter("motor_id").value
        self._spin_up_time   = self.get_parameter("spin_up_time_s").value
        self._spin_down_time = self.get_parameter("spin_down_time_s").value
        self._step_time      = self.get_parameter("step_time_s").value

        self._status_pub = self.create_publisher(String, "motor_status", 10)
        self.create_subscription(Float32, "motor_spin", self._on_spin, 10)
        self.create_subscription(Empty,   "motor_stop", self._on_stop, 10)
        self.create_subscription(Empty,   "motor_step", self._on_step, 10)

        self._current_rpm = 0.0
        self._lock = threading.Lock()

        self._publish_status("idle")
        self.get_logger().info(
            f"[{self._motor_id}] Rotary stub ready  "
            f"(spin_up={self._spin_up_time}s  step={self._step_time}s)"
        )
        self.get_logger().warn(
            f"[{self._motor_id}] STUB — no hardware. Replace with real driver when motor arrives."
        )

    # ── Callbacks ──────────────────────────────

    def _on_spin(self, msg: Float32):
        rpm = msg.data
        with self._lock:
            self._current_rpm = rpm
        self.get_logger().info(f"[{self._motor_id}] Spinning at {rpm:.1f} rpm (simulated)")

        def _delayed_running():
            self._publish_status("spinning")

        threading.Timer(self._spin_up_time, _delayed_running).start()

    def _on_stop(self, _):
        with self._lock:
            self._current_rpm = 0.0
        self.get_logger().info(f"[{self._motor_id}] Stopping (simulated)")

        def _delayed_idle():
            self._publish_status("idle")

        threading.Timer(self._spin_down_time, _delayed_idle).start()

    def _on_step(self, _):
        """Conveyor: advance one tray, publish 'stepping' then 'idle' after step_time_s."""
        self.get_logger().info(f"[{self._motor_id}] Step command — advancing one tray (simulated {self._step_time}s)")
        self._publish_status("stepping")
        threading.Timer(self._step_time, lambda: self._publish_status("idle")).start()

    # ── Helpers ────────────────────────────────

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f"[{self._motor_id}] status → {status}")


def main():
    rclpy.init()
    node = RotaryMotorStubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
