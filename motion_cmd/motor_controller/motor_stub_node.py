"""
Motor stub node — simulates all motors that have not yet arrived.

Use this node to run and test the full ROS2 graph (peel_sequence_node,
motor_controller_node, etc.) with only the one real DR28T motor connected.
Each instance is launched in its own namespace so a single binary serves
every pending motor.

Replace this stub with the real driver node when each motor + controller
arrives and is wired up.

--------------------------------------------------------------------
Type 1 — Linear position control  (M① M③ M④ M⑥)
--------------------------------------------------------------------
  ~/motor_cmd    [std_msgs/Float32MultiArray]
                  data[0]  target_position_mm
                  data[1]  speed_mm_s
                  data[2]  accel_mm_s2    (optional; 0 → node default)
                  data[3]  decel_mm_s2    (optional; 0 → same as accel)
                  data[4]  move_time_s    (optional; overrides sim time when > 0)
  ~/motor_home   [std_msgs/Empty]   → simulated ZHOME sequence

Type 2 — Rotary velocity control  (M② M⑤)
--------------------------------------------------------------------
  ~/motor_spin   [std_msgs/Float32MultiArray]
                  data[0]  target_rpm
                  data[1]  accel_rpm_s   (optional; 0 → instant)
                  data[2]  decel_rpm_s   (optional; 0 → same as accel)
  ~/motor_stop   [std_msgs/Empty]   → stop and hold

Published topics
--------------------------------------------------------------------
  ~/motor_status [std_msgs/String]   — "idle" | "running" | "spinning"

Parameters
--------------------------------------------------------------------
  motor_id          (str)   — label used in log messages
  spin_up_time_s    (float) — simulated spin-up delay before "spinning"  (default 0.3)
  spin_down_time_s  (float) — simulated spin-down delay before "idle"    (default 0.2)
  move_time_s       (float) — simulated travel time before "idle"        (default 1.0)
  move_settle_s     (float) — extra settle delay after move completes    (default 0.1)

Rotary position (Motor⑤ only)
--------------------------------------------------------------------
  ~/motor_angle_cmd [std_msgs/Float32MultiArray]
                  data[0]  target_angle_deg
                  data[1]  speed_rpm
                  data[2]  accel_rpm_s   (optional)
                  data[3]  decel_rpm_s   (optional)
  ~/motor_home is shared with Type 1 — returns rotary axis to 0°
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Empty, String


class MotorStubNode(Node):

    def __init__(self):
        super().__init__("motor_stub")

        self.declare_parameter("motor_id",         "motor")
        self.declare_parameter("spin_up_time_s",   0.3)
        self.declare_parameter("spin_down_time_s", 0.2)
        self.declare_parameter("move_time_s",      1.0)
        self.declare_parameter("move_settle_s",    0.1)

        self._motor_id       = self.get_parameter("motor_id").value
        self._spin_up_time   = self.get_parameter("spin_up_time_s").value
        self._spin_down_time = self.get_parameter("spin_down_time_s").value
        self._move_time      = self.get_parameter("move_time_s").value
        self._move_settle    = self.get_parameter("move_settle_s").value

        self._status_pub = self.create_publisher(String, "motor_status", 10)

        # Type 1 — linear position control
        self.create_subscription(Float32MultiArray, "motor_cmd",       self._on_cmd,       10)
        self.create_subscription(Empty,             "motor_home",      self._on_home,      10)

        # Type 2 — rotary velocity control
        self.create_subscription(Float32MultiArray, "motor_spin",      self._on_spin,      10)
        self.create_subscription(Empty,             "motor_stop",      self._on_stop,      10)

        # Rotary position control (Motor⑤ peeler orbit — angle positioning)
        self.create_subscription(Float32MultiArray, "motor_angle_cmd", self._on_angle_cmd, 10)

        self._current_pos_mm = 0.0
        self._current_rpm    = 0.0
        self._lock           = threading.Lock()

        self._publish_status("idle")
        self.get_logger().info(
            f"[{self._motor_id}] Motor stub ready  "
            f"(spin_up={self._spin_up_time}s  move={self._move_time}s)"
        )
        self.get_logger().warn(
            f"[{self._motor_id}] STUB — no hardware.  "
            f"Replace with the real driver node when this motor arrives."
        )

    # ── Type 1 callbacks ───────────────────────────────────────────

    def _on_cmd(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn(
                f"[{self._motor_id}] motor_cmd needs at least [pos_mm, speed_mms]"
            )
            return
        pos_mm = msg.data[0]
        speed  = msg.data[1]
        accel  = msg.data[2] if len(msg.data) > 2 else 0.0
        decel  = msg.data[3] if len(msg.data) > 3 else accel
        move_t = msg.data[4] if len(msg.data) > 4 else 0.0

        with self._lock:
            self._current_pos_mm = pos_mm

        sim_time = move_t if move_t > 0.0 else self._move_time
        self.get_logger().info(
            f"[{self._motor_id}] CMD  pos={pos_mm:.1f} mm  spd={speed:.1f} mm/s  "
            f"accel={accel:.1f}  decel={decel:.1f}  (sim {sim_time:.2f}s)"
        )
        self._publish_status("running")
        threading.Timer(
            sim_time + self._move_settle,
            lambda: self._publish_status("idle"),
        ).start()

    def _on_home(self, _):
        with self._lock:
            self._current_pos_mm = 0.0
        self.get_logger().info(
            f"[{self._motor_id}] HOME — returning to zero (sim {self._move_time:.2f}s)"
        )
        self._publish_status("running")
        threading.Timer(
            self._move_time + self._move_settle,
            lambda: self._publish_status("idle"),
        ).start()

    def _on_angle_cmd(self, msg: Float32MultiArray):
        """Rotary absolute position command: [angle_deg, speed_rpm, accel?, decel?]."""
        if len(msg.data) < 2:
            self.get_logger().warn(
                f"[{self._motor_id}] motor_angle_cmd needs at least [angle_deg, speed_rpm]"
            )
            return
        angle_deg = msg.data[0]
        speed_rpm = msg.data[1]
        accel     = msg.data[2] if len(msg.data) > 2 else 0.0
        decel     = msg.data[3] if len(msg.data) > 3 else accel

        self.get_logger().info(
            f"[{self._motor_id}] ANGLE  target={angle_deg:.1f}°  spd={speed_rpm:.1f} rpm  "
            f"accel={accel:.1f}  decel={decel:.1f}  (sim {self._move_time:.2f}s)"
        )
        self._publish_status("running")
        threading.Timer(
            self._move_time + self._move_settle,
            lambda: self._publish_status("idle"),
        ).start()

    # ── Type 2 callbacks ───────────────────────────────────────────

    def _on_spin(self, msg: Float32MultiArray):
        if len(msg.data) < 1:
            self.get_logger().warn(
                f"[{self._motor_id}] motor_spin needs at least [rpm]"
            )
            return
        rpm   = msg.data[0]
        accel = msg.data[1] if len(msg.data) > 1 else 0.0
        decel = msg.data[2] if len(msg.data) > 2 else accel

        with self._lock:
            self._current_rpm = rpm

        self.get_logger().info(
            f"[{self._motor_id}] SPIN  rpm={rpm:.1f}  accel={accel:.1f}  "
            f"decel={decel:.1f}  (simulated)"
        )
        threading.Timer(
            self._spin_up_time,
            lambda: self._publish_status("spinning"),
        ).start()

    def _on_stop(self, _):
        with self._lock:
            self._current_rpm = 0.0
        self.get_logger().info(f"[{self._motor_id}] STOP (simulated)")
        threading.Timer(
            self._spin_down_time,
            lambda: self._publish_status("idle"),
        ).start()

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
