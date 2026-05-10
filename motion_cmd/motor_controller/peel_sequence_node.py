"""
Peel sequence node — orchestrates all 6 motors for one yuzu peel cycle.

Machine spec (YuzuSequence.pdf):
  Motor①  Yuzu holding set       — Z-axis linear actuator
  Motor②  Rotational motion      — 200-250 rpm, CCW during peeling
  Motor③  Feed motion (top)      — peeling depth, linear
  Motor④  Feed motion (bottom)   — peeling depth, linear
  Motor⑤  Rotational motion      — 20 rpm; positions 90° then CW orbit
  Motor⑥  Conveyor feed

Sequence (YuzuSequence.pdf page 2):
  Step 1  Yuzu positioning   Motor⑥  FD
  Step 2  Yuzu placement     Motor①  Approach
  Step 3  Peeler positioning Motor⑤  90° ROT
  Step 4  Rotation + feed    Motor② ① CCW ROT
                             Motor③&④ ② FD  (simultaneous)
                             Motor⑤  ③ CW ROT
  Step 5  Yuzu gripping      Motor③&④ FD  (additional grip)
  Step 6  Feed motion        Motor①  HOME  (+ stop ② and ⑤)
  Step 7  Yuzu release       Motor③&④ HOME
  Step 8  Waiting for next cycle

=============================================================
IMAGE TEAM INTERFACE
=============================================================
Step 1 — publish yuzu measurements (optional; improves depth adaptation):
  Topic : /yuzu_detected
  Type  : std_msgs/Float32MultiArray
  data  : [x_offset_mm, y_offset_mm, long_axis_mm, short_axis_mm]
    x_offset_mm   — lateral offset of yuzu centre from nominal
    y_offset_mm   — reserved
    long_axis_mm  — scales Z travel  (nominal 50 mm)
    short_axis_mm — scales peeler advance depth  (nominal 44 mm)

Step 2 — trigger the cycle:
  Topic : /peel/start
  Type  : std_msgs/Empty

Monitor:
  Topic : /peel/status
  Type  : std_msgs/String
  values: idle | yuzu_positioning | yuzu_placement | peeler_positioning |
          rotation_feed | yuzu_gripping | z_retracting | yuzu_release |
          done | error | aborted

Emergency stop:
  Topic : /peel/abort
  Type  : std_msgs/Empty
=============================================================

All timing and motion values are ROS2 parameters — tune without recompiling:
  ros2 param set /peel_sequence peel_duration_s 1.2
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Empty, String

YUZU_LONG_NOM_MM  = 50.0
YUZU_SHORT_NOM_MM = 44.0


class PeelSequenceNode(Node):

    def __init__(self):
        super().__init__("peel_sequence")

        # ── Motion / timing parameters ──
        self.declare_parameter("conveyor_settle_s",          1.2)
        self.declare_parameter("z_lower_pos_mm",            25.0)
        self.declare_parameter("z_lower_speed_mms",         15.0)
        self.declare_parameter("z_lower_wait_s",             2.0)
        # Peeler positioning: Motor⑤ spins for this long to reach 90°
        # 0.25 rev at 20 rpm = 0.75 s
        self.declare_parameter("peeler_position_rot_s",      0.75)
        self.declare_parameter("peeler_position_wait_s",     0.3)
        # Step 4 — rotation + feed
        self.declare_parameter("yuzu_rotation_rpm",        225.0)   # Motor② 200-250 rpm
        self.declare_parameter("spin_up_wait_s",             0.3)
        self.declare_parameter("peeler_advance_mm",         10.0)   # Motor③&④ FD depth
        self.declare_parameter("peeler_advance_speed_mms",   8.0)
        self.declare_parameter("peeler_advance_wait_s",      1.0)
        self.declare_parameter("peeler_orbit_rpm",          20.0)   # Motor⑤ 20 rpm
        self.declare_parameter("peel_duration_s",            1.5)
        # Step 5 — yuzu gripping (extra FD on top of advance)
        self.declare_parameter("grip_advance_mm",            3.0)
        self.declare_parameter("grip_speed_mms",             5.0)
        self.declare_parameter("grip_wait_s",                0.5)
        # Step 6 — Z home
        self.declare_parameter("z_home_wait_s",              2.0)
        # Step 7 — peeler release
        self.declare_parameter("release_wait_s",             0.8)

        # ── Publishers to all 6 motors ──
        self._pub = {
            # Motor①: Z-axis (yuzu holding set)
            "z_cmd":          self.create_publisher(Float32MultiArray, "/motor_z/motor_cmd",  10),
            "z_home":         self.create_publisher(Empty,             "/motor_z/motor_home", 10),
            # Motor②: Yuzu rotation spindle (CCW, 200-250 rpm)
            "yuzu_spin":      self.create_publisher(Float32,           "/motor_yuzu_rot/motor_spin", 10),
            "yuzu_stop":      self.create_publisher(Empty,             "/motor_yuzu_rot/motor_stop", 10),
            # Motor③: Peeler feed top (linear, peeling depth)
            "peeler3_cmd":    self.create_publisher(Float32MultiArray, "/motor_peeler_3/motor_cmd",  10),
            "peeler3_home":   self.create_publisher(Empty,             "/motor_peeler_3/motor_home", 10),
            # Motor④: Peeler feed bottom (linear, peeling depth)
            "peeler4_cmd":    self.create_publisher(Float32MultiArray, "/motor_peeler_4/motor_cmd",  10),
            "peeler4_home":   self.create_publisher(Empty,             "/motor_peeler_4/motor_home", 10),
            # Motor⑤: Peeler orbit (positions 90° then CW at 20 rpm)
            "orbit_spin":     self.create_publisher(Float32,           "/motor_peeler_orbit/motor_spin", 10),
            "orbit_stop":     self.create_publisher(Empty,             "/motor_peeler_orbit/motor_stop", 10),
            # Motor⑥: Conveyor
            "conveyor_step":  self.create_publisher(Empty,             "/motor_conveyor/motor_step", 10),
        }

        # ── Peel sequence interface ──
        self._status_pub = self.create_publisher(String, "/peel/status", 10)
        self.create_subscription(Empty,             "/peel/start",    self._on_start,         10)
        self.create_subscription(Empty,             "/peel/abort",    self._on_abort,         10)
        self.create_subscription(Float32MultiArray, "/yuzu_detected", self._on_yuzu_detected, 10)

        self._state       = "idle"
        self._abort_event = threading.Event()
        self._seq_thread  = None
        self._yuzu_info   = None

        self._publish_status("idle")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Peel sequence node ready.  6-motor / 8-step (YuzuSequence.pdf)")
        self.get_logger().info("IMAGE TEAM: publish /yuzu_detected then /peel/start")
        self.get_logger().info("  /yuzu_detected  Float32MultiArray  [x_off, y_off, long_mm, short_mm]")
        self.get_logger().info("  /peel/start     Empty")
        self.get_logger().info("  /peel/abort     Empty")
        self.get_logger().info("  /peel/status    String  (output)")
        self.get_logger().info("=" * 60)

    # ── External callbacks ─────────────────────────────────────────

    def _on_yuzu_detected(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn("/yuzu_detected needs 4 values: [x_off, y_off, long_mm, short_mm]")
            return
        self._yuzu_info = {
            "x_offset_mm":   float(msg.data[0]),
            "y_offset_mm":   float(msg.data[1]),
            "long_axis_mm":  float(msg.data[2]),
            "short_axis_mm": float(msg.data[3]),
        }
        self.get_logger().info(
            f"Yuzu detected — long={self._yuzu_info['long_axis_mm']:.1f} mm  "
            f"short={self._yuzu_info['short_axis_mm']:.1f} mm  "
            f"offset=({self._yuzu_info['x_offset_mm']:.1f}, {self._yuzu_info['y_offset_mm']:.1f}) mm"
        )

    def _on_start(self, _):
        if self._state not in ("idle", "done", "error", "aborted"):
            self.get_logger().warn(f"Busy (state={self._state}) — ignoring /peel/start.")
            return
        self.get_logger().info("Starting peel sequence.")
        self._abort_event.clear()
        self._seq_thread = threading.Thread(target=self._run_sequence, daemon=True)
        self._seq_thread.start()

    def _on_abort(self, _):
        self.get_logger().warn("ABORT received — stopping all motors immediately.")
        self._abort_event.set()
        self._emergency_stop()
        self._set_state("aborted")

    # ── Sequence state machine ─────────────────────────────────────

    def _run_sequence(self):
        try:
            p = self._load_params()

            # Adapt depths to detected yuzu size
            z_lower       = p["z_lower_pos_mm"]
            blade_advance = p["peeler_advance_mm"]
            if self._yuzu_info:
                z_lower       = min(29.0, z_lower * (self._yuzu_info["long_axis_mm"]  / YUZU_LONG_NOM_MM))
                blade_advance = min(29.0, blade_advance * (self._yuzu_info["short_axis_mm"] / YUZU_SHORT_NOM_MM))
                self.get_logger().info(
                    f"Adapted — z_lower={z_lower:.1f} mm  blade_advance={blade_advance:.1f} mm"
                )

            # Step 1: Yuzu positioning — Motor⑥ FD
            self._set_state("yuzu_positioning")
            self._pub["conveyor_step"].publish(Empty())
            if not self._wait(p["conveyor_settle_s"]): return

            # Step 2: Yuzu placement — Motor① Approach (Z-axis lowers onto yuzu)
            self._set_state("yuzu_placement")
            self._send_linear("z_cmd", z_lower, p["z_lower_speed_mms"])
            if not self._wait(p["z_lower_wait_s"]): return

            # Step 3: Peeler positioning — Motor⑤ 90° ROT
            # Spin at orbit rpm for timed duration to reach 90° (0.25 rev at 20 rpm = 0.75 s)
            self._set_state("peeler_positioning")
            self._send_rpm("orbit_spin", p["peeler_orbit_rpm"])
            if not self._wait(p["peeler_position_rot_s"]): return
            self._pub["orbit_stop"].publish(Empty())
            if not self._wait(p["peeler_position_wait_s"]): return

            # Step 4: Rotation and feed motion — three sequential sub-steps per spec
            self._set_state("rotation_feed")
            # ① Motor② CCW ROT — yuzu starts spinning
            self._send_rpm("yuzu_spin", p["yuzu_rotation_rpm"])
            if not self._wait(p["spin_up_wait_s"]): return
            # ② Motor③&④ FD simultaneously — peelers advance to peeling depth
            self._send_linear("peeler3_cmd", blade_advance, p["peeler_advance_speed_mms"])
            self._send_linear("peeler4_cmd", blade_advance, p["peeler_advance_speed_mms"])
            if not self._wait(p["peeler_advance_wait_s"]): return
            # ③ Motor⑤ CW ROT — peeler orbit starts; hold for peel_duration_s
            self._send_rpm("orbit_spin", p["peeler_orbit_rpm"])
            if not self._wait(p["peel_duration_s"]): return

            # Step 5: Yuzu gripping — Motor③&④ FD (additional grip beyond peeling depth)
            self._set_state("yuzu_gripping")
            grip_pos = min(29.0, blade_advance + p["grip_advance_mm"])
            self._send_linear("peeler3_cmd", grip_pos, p["grip_speed_mms"])
            self._send_linear("peeler4_cmd", grip_pos, p["grip_speed_mms"])
            if not self._wait(p["grip_wait_s"]): return

            # Step 6: Feed motion — Motor① HOME; also stop ② and ⑤
            self._set_state("z_retracting")
            self._pub["z_home"].publish(Empty())
            self._pub["yuzu_stop"].publish(Empty())
            self._pub["orbit_stop"].publish(Empty())
            if not self._wait(p["z_home_wait_s"]): return

            # Step 7: Yuzu release — Motor③&④ HOME
            self._set_state("yuzu_release")
            self._pub["peeler3_home"].publish(Empty())
            self._pub["peeler4_home"].publish(Empty())
            if not self._wait(p["release_wait_s"]): return

            self._set_state("done")

        except Exception as exc:
            self.get_logger().error(f"Sequence exception: {exc}")
            self._emergency_stop()
            self._set_state("error")

    # ── Helpers ────────────────────────────────────────────────────

    def _load_params(self) -> dict:
        names = [
            "conveyor_settle_s",
            "z_lower_pos_mm", "z_lower_speed_mms", "z_lower_wait_s",
            "peeler_position_rot_s", "peeler_position_wait_s",
            "yuzu_rotation_rpm", "spin_up_wait_s",
            "peeler_advance_mm", "peeler_advance_speed_mms", "peeler_advance_wait_s",
            "peeler_orbit_rpm", "peel_duration_s",
            "grip_advance_mm", "grip_speed_mms", "grip_wait_s",
            "z_home_wait_s", "release_wait_s",
        ]
        return {n: self.get_parameter(n).value for n in names}

    def _send_linear(self, key: str, pos_mm: float, speed_mms: float):
        msg = Float32MultiArray()
        msg.data = [float(pos_mm), float(speed_mms)]
        self._pub[key].publish(msg)

    def _send_rpm(self, key: str, rpm: float):
        msg = Float32()
        msg.data = float(rpm)
        self._pub[key].publish(msg)

    def _wait(self, duration_s: float) -> bool:
        """Block for duration_s; return False early if abort received."""
        aborted = self._abort_event.wait(timeout=duration_s)
        return not aborted

    def _emergency_stop(self):
        self._pub["yuzu_stop"].publish(Empty())
        self._pub["orbit_stop"].publish(Empty())
        self._pub["z_home"].publish(Empty())
        self._pub["peeler3_home"].publish(Empty())
        self._pub["peeler4_home"].publish(Empty())
        self.get_logger().warn("Emergency stop: all motors halted / homed.")

    def _set_state(self, state: str):
        self._state = state
        self._publish_status(state)
        self.get_logger().info(f"Peel state → {state}")

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    node = PeelSequenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
