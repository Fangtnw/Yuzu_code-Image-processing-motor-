"""
Motor stub node — simulates all motors that have not yet arrived.

Use this node to run and test the full ROS2 graph (peel_sequence_node,
motor_controller_node, etc.) with only the one real DR28T motor connected.
Each instance is launched in its own namespace so a single binary serves
every pending motor.

Replace this stub with the real driver node when each motor + controller
arrives and is wired up.

--------------------------------------------------------------------
Subscribed topics  (relative to node namespace)
--------------------------------------------------------------------
  ~/motor_spin   [std_msgs/Float32]  — set target RPM  (rotary motors)
  ~/motor_stop   [std_msgs/Empty]    — stop rotation
  ~/motor_step   [std_msgs/Empty]    — advance one position (conveyor)

Published topics
--------------------------------------------------------------------
  ~/motor_status [std_msgs/String]   — "idle" | "spinning" | "stepping"

Parameters
--------------------------------------------------------------------
  motor_id          (str)   — label used in log messages
  spin_up_time_s    (float) — simulated delay before "spinning"   (default 0.3)
  spin_down_time_s  (float) — simulated delay before "idle"       (default 0.2)
  step_time_s       (float) — simulated conveyor step duration    (default 1.5)
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Empty, String


class MotorStubNode(Node):

    def __init__(self):
        super().__init__("motor_stub")

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
            f"[{self._motor_id}] Motor stub ready  "
            f"(spin_up={self._spin_up_time}s  step={self._step_time}s)"
        )
        self.get_logger().warn(
            f"[{self._motor_id}] STUB — no hardware.  "
            f"Replace with the real driver node when this motor arrives."
        )

    # ── Callbacks ──────────────────────────────────────────────────

    def _on_spin(self, msg: Float32):
        rpm = msg.data
        with self._lock:
            self._current_rpm = rpm
        self.get_logger().info(f"[{self._motor_id}] Spinning at {rpm:.1f} rpm (simulated)")
        threading.Timer(self._spin_up_time, lambda: self._publish_status("spinning")).start()

    def _on_stop(self, _):
        with self._lock:
            self._current_rpm = 0.0
        self.get_logger().info(f"[{self._motor_id}] Stopping (simulated)")
        threading.Timer(self._spin_down_time, lambda: self._publish_status("idle")).start()

    def _on_step(self, _):
        """Conveyor: advance one tray, then return to idle after step_time_s."""
        self.get_logger().info(
            f"[{self._motor_id}] Step command — advancing one tray "
            f"(simulated {self._step_time}s)"
        )
        self._publish_status("stepping")
        threading.Timer(self._step_time, lambda: self._publish_status("idle")).start()

    # ── Helpers ────────────────────────────────────────────────────

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f"[{self._motor_id}] status → {status}")


def main():
    rclpy.init()
    node = MotorStubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
