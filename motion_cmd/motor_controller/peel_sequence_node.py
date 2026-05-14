"""
Peel sequence node — orchestrates all 6 motors for one yuzu peel cycle.

Machine spec (YuzuSequence.pdf):
  Motor①  Yuzu holding set       — Z-axis linear actuator
  Motor②  Rotational motion      — 200-250 rpm, CCW during peeling
  Motor③  Feed motion (top)      — peeling depth, linear
  Motor④  Feed motion (bottom)   — peeling depth, linear
  Motor⑤  Rotational motion      — 20 rpm; positions 90° then CW orbit
  Motor⑥  Conveyor feed          — Type 1 linear (cumulative position per tray)

Sequence (YuzuSequence.pdf page 2):
  Step 1  Yuzu positioning   Motor⑥  FD  (motor_cmd: cumulative tray position)
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

Step 2 — trigger the cycle (choose one mode):
  Auto mode  : /peel/start        Empty  — runs all 7 steps unattended
  Manual mode: /peel/start_manual Empty  — pauses after each step completes;
               /peel/next_step    Empty  — advance to the next step
                                           (ignored unless status == manual_pause)

Monitor:
  Topic : /peel/status
  Type  : std_msgs/String
  values: idle | yuzu_positioning | yuzu_placement | peeler_positioning |
          rotation_feed | yuzu_gripping | z_retracting | yuzu_release |
          manual_pause | done | error | aborted

Emergency stop:
  Topic : /peel/abort
  Type  : std_msgs/Empty
=============================================================

Motor interface types:
  Type 1 (linear) — motor_cmd  [Float32MultiArray: pos_mm, speed_mms, accel?, decel?, time?]
                  — motor_home [Empty]
                  Motors: ① motor_z, ③ motor_peeler_3, ④ motor_peeler_4, ⑥ motor_conveyor
  Type 2 (rotary) — motor_spin [Float32MultiArray: rpm, accel_rpm_s, decel_rpm_s]
                  — motor_stop [Empty]
                  Motors: ② motor_yuzu_rot, ⑤ motor_peeler_orbit

All timing and motion values are ROS2 parameters — tune without recompiling:
  ros2 param set /peel_sequence peel_duration_s 1.2
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Empty, String

YUZU_LONG_NOM_MM  = 50.0
YUZU_SHORT_NOM_MM = 44.0


class PeelSequenceNode(Node):

    def __init__(self):
        super().__init__("peel_sequence")

        # ── Motion / timing parameters ──
        # Conveyor (Motor⑥) — Type 1; advances by one tray pitch per cycle
        self.declare_parameter("conveyor_settle_s",           1.2)
        self.declare_parameter("conveyor_advance_mm",        80.0)   # one tray pitch
        self.declare_parameter("conveyor_speed_mms",         20.0)
        self.declare_parameter("conveyor_accel_mms2",         1.0)
        self.declare_parameter("conveyor_decel_mms2",         1.0)
        # Z-axis (Motor①) — approach / home
        self.declare_parameter("z_lower_pos_mm",             25.0)
        self.declare_parameter("z_lower_speed_mms",          15.0)
        self.declare_parameter("z_accel_mms2",                1.0)
        self.declare_parameter("z_decel_mms2",                1.0)
        self.declare_parameter("z_lower_wait_s",              2.0)
        # Peeler positioning (Motor⑤): angle position command to reach 90°
        self.declare_parameter("peeler_position_angle_deg",  90.0)   # target angle for Step 3
        self.declare_parameter("peeler_position_wait_s",      1.0)   # time to complete 90° move + settle
        # Step 4 — rotation + feed
        self.declare_parameter("yuzu_rotation_rpm",         225.0)   # Motor② 200-250 rpm
        self.declare_parameter("yuzu_rotation_accel_rpm_s", 750.0)
        self.declare_parameter("yuzu_rotation_decel_rpm_s", 750.0)
        self.declare_parameter("spin_up_wait_s",              0.3)
        self.declare_parameter("peeler_advance_mm",          10.0)   # Motor③&④ FD depth
        self.declare_parameter("peeler_advance_speed_mms",    8.0)
        self.declare_parameter("peeler_feed_accel_mms2",      1.0)
        self.declare_parameter("peeler_feed_decel_mms2",      1.0)
        self.declare_parameter("peeler_advance_wait_s",       1.0)
        self.declare_parameter("peeler_orbit_rpm",           20.0)   # Motor⑤ 20 rpm
        self.declare_parameter("peeler_orbit_accel_rpm_s",  100.0)
        self.declare_parameter("peeler_orbit_decel_rpm_s",  100.0)
        self.declare_parameter("peel_duration_s",             1.5)
        # Step 5 — yuzu gripping (extra FD on top of advance)
        self.declare_parameter("grip_advance_mm",             3.0)
        self.declare_parameter("grip_speed_mms",              5.0)
        self.declare_parameter("grip_wait_s",                 0.5)
        # Step 6 — Z home
        self.declare_parameter("z_home_wait_s",               2.0)
        # Step 7 — peeler release
        self.declare_parameter("release_wait_s",              0.8)

        # ── Publishers to all 6 motors ──
        self._pub = {
            # Motor①: Z-axis (yuzu holding set) — Type 1
            "z_cmd":         self.create_publisher(Float32MultiArray, "/motor_z/motor_cmd",  10),
            "z_home":        self.create_publisher(Empty,             "/motor_z/motor_home", 10),
            # Motor②: Yuzu rotation spindle — Type 2 [rpm, accel_rpm_s, decel_rpm_s]
            "yuzu_spin":     self.create_publisher(Float32MultiArray, "/motor_yuzu_rot/motor_spin", 10),
            "yuzu_stop":     self.create_publisher(Empty,             "/motor_yuzu_rot/motor_stop", 10),
            # Motor③: Peeler feed top — Type 1
            "peeler3_cmd":   self.create_publisher(Float32MultiArray, "/motor_peeler_3/motor_cmd",  10),
            "peeler3_home":  self.create_publisher(Empty,             "/motor_peeler_3/motor_home", 10),
            # Motor④: Peeler feed bottom — Type 1
            "peeler4_cmd":   self.create_publisher(Float32MultiArray, "/motor_peeler_4/motor_cmd",  10),
            "peeler4_home":  self.create_publisher(Empty,             "/motor_peeler_4/motor_home", 10),
            # Motor⑤: Peeler orbit — hybrid: angle position (Step 3) + velocity (Step 4)
            #   motor_angle_cmd [Float32MultiArray: angle_deg, speed_rpm, accel_rpm_s?, decel_rpm_s?]
            #   motor_home      [Empty]  → return to 0°
            #   motor_spin      [Float32MultiArray: rpm, accel_rpm_s, decel_rpm_s]
            #   motor_stop      [Empty]
            "orbit_angle_cmd": self.create_publisher(Float32MultiArray, "/motor_peeler_orbit/motor_angle_cmd", 10),
            "orbit_home":      self.create_publisher(Empty,             "/motor_peeler_orbit/motor_home",      10),
            "orbit_spin":      self.create_publisher(Float32MultiArray, "/motor_peeler_orbit/motor_spin",      10),
            "orbit_stop":      self.create_publisher(Empty,             "/motor_peeler_orbit/motor_stop",      10),
            # Motor⑥: Conveyor — Type 1; cumulative position tracks tray count
            "conveyor_cmd":  self.create_publisher(Float32MultiArray, "/motor_conveyor/motor_cmd",  10),
            "conveyor_home": self.create_publisher(Empty,             "/motor_conveyor/motor_home", 10),
        }

        # ── Peel sequence interface ──
        self._status_pub = self.create_publisher(String, "/peel/status", 10)
        self.create_subscription(Empty,             "/peel/start",        self._on_start,         10)
        self.create_subscription(Empty,             "/peel/start_manual", self._on_start_manual,  10)
        self.create_subscription(Empty,             "/peel/next_step",    self._on_next_step,     10)
        self.create_subscription(Empty,             "/peel/abort",        self._on_abort,         10)
        self.create_subscription(Float32MultiArray, "/yuzu_detected",     self._on_yuzu_detected, 10)

        self._state           = "idle"
        self._abort_event     = threading.Event()
        self._step_event      = threading.Event()   # gates manual mode between steps
        self._manual_mode     = False
        self._seq_thread      = None
        self._yuzu_info       = None
        self._conveyor_pos_mm = 0.0   # cumulative conveyor position (n_trays × pitch)

        self._publish_status("idle")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Peel sequence node ready.  6-motor / 8-step (YuzuSequence.pdf)")
        self.get_logger().info("  Auto mode  : /peel/start")
        self.get_logger().info("  Manual mode: /peel/start_manual  then  /peel/next_step per step")
        self.get_logger().info("  Abort      : /peel/abort")
        self.get_logger().info("  Status     : /peel/status")
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
        self.get_logger().info("Starting peel sequence  [AUTO mode].")
        self._manual_mode = False
        self._abort_event.clear()
        self._step_event.clear()
        self._seq_thread = threading.Thread(target=self._run_sequence, daemon=True)
        self._seq_thread.start()

    def _on_start_manual(self, _):
        if self._state not in ("idle", "done", "error", "aborted"):
            self.get_logger().warn(f"Busy (state={self._state}) — ignoring /peel/start_manual.")
            return
        self.get_logger().info("Starting peel sequence  [MANUAL step-by-step mode].")
        self._manual_mode = True
        self._abort_event.clear()
        self._step_event.clear()
        self._seq_thread = threading.Thread(target=self._run_sequence, daemon=True)
        self._seq_thread.start()

    def _on_next_step(self, _):
        if self._state != "manual_pause":
            self.get_logger().warn(
                f"Not paused (state={self._state}) — ignoring /peel/next_step.")
            return
        self.get_logger().info("Next step confirmed by operator.")
        self._step_event.set()

    def _on_abort(self, _):
        self.get_logger().warn("ABORT received — stopping all motors immediately.")
        self._abort_event.set()
        self._step_event.set()   # unblock any waiting gate so thread can exit
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

            # ── Step 1: Yuzu positioning — Motor⑥ FD ──────────────────
            self._set_state("yuzu_positioning")
            self._conveyor_pos_mm += p["conveyor_advance_mm"]
            self._send_linear(
                "conveyor_cmd",
                self._conveyor_pos_mm,
                p["conveyor_speed_mms"],
                p["conveyor_accel_mms2"],
                p["conveyor_decel_mms2"],
            )
            if not self._wait_step(p["conveyor_settle_s"]): return

            # ── Step 2: Yuzu placement — Motor① Approach ──────────────
            self._set_state("yuzu_placement")
            self._send_linear(
                "z_cmd",
                z_lower,
                p["z_lower_speed_mms"],
                p["z_accel_mms2"],
                p["z_decel_mms2"],
            )
            if not self._wait_step(p["z_lower_wait_s"]): return

            # ── Step 3: Peeler positioning — Motor⑤ 90° ROT ───────────
            # Use absolute angle command (ABZO encoder) — reliable vs. timed velocity
            self._set_state("peeler_positioning")
            self._send_angle("orbit_angle_cmd", p["peeler_position_angle_deg"],
                             p["peeler_orbit_rpm"],
                             p["peeler_orbit_accel_rpm_s"], p["peeler_orbit_decel_rpm_s"])
            if not self._wait_step(p["peeler_position_wait_s"]): return

            # ── Step 4: Rotation + feed — ①②③ sub-steps ──────────────
            self._set_state("rotation_feed")
            # ① Motor② CCW ROT
            self._send_rpm("yuzu_spin", p["yuzu_rotation_rpm"],
                           p["yuzu_rotation_accel_rpm_s"], p["yuzu_rotation_decel_rpm_s"])
            if not self._wait(p["spin_up_wait_s"]): return          # intra-step timing
            # ② Motor③&④ FD simultaneously
            self._send_linear(
                "peeler3_cmd",
                blade_advance,
                p["peeler_advance_speed_mms"],
                p["peeler_feed_accel_mms2"],
                p["peeler_feed_decel_mms2"],
            )
            self._send_linear(
                "peeler4_cmd",
                blade_advance,
                p["peeler_advance_speed_mms"],
                p["peeler_feed_accel_mms2"],
                p["peeler_feed_decel_mms2"],
            )
            if not self._wait(p["peeler_advance_wait_s"]): return   # intra-step timing
            # ③ Motor⑤ CW ROT — orbit for peel_duration_s
            self._send_rpm("orbit_spin", p["peeler_orbit_rpm"],
                           p["peeler_orbit_accel_rpm_s"], p["peeler_orbit_decel_rpm_s"])
            if not self._wait_step(p["peel_duration_s"]): return

            # ── Step 5: Yuzu gripping — Motor③&④ FD ──────────────────
            self._set_state("yuzu_gripping")
            grip_pos = min(29.0, blade_advance + p["grip_advance_mm"])
            self._send_linear(
                "peeler3_cmd",
                grip_pos,
                p["grip_speed_mms"],
                p["peeler_feed_accel_mms2"],
                p["peeler_feed_decel_mms2"],
            )
            self._send_linear(
                "peeler4_cmd",
                grip_pos,
                p["grip_speed_mms"],
                p["peeler_feed_accel_mms2"],
                p["peeler_feed_decel_mms2"],
            )
            if not self._wait_step(p["grip_wait_s"]): return

            # ── Step 6: Z retract — Motor① HOME + stop ②⑤ ────────────
            self._set_state("z_retracting")
            self._pub["z_home"].publish(Empty())
            self._pub["yuzu_stop"].publish(Empty())
            self._pub["orbit_stop"].publish(Empty())
            if not self._wait_step(p["z_home_wait_s"]): return

            # ── Step 7: Yuzu release — Motor③&④ HOME ──────────────────
            self._set_state("yuzu_release")
            self._pub["peeler3_home"].publish(Empty())
            self._pub["peeler4_home"].publish(Empty())
            if not self._wait_step(p["release_wait_s"]): return

            self._set_state("done")

        except Exception as exc:
            self.get_logger().error(f"Sequence exception: {exc}")
            self._emergency_stop()
            self._set_state("error")

    # ── Helpers ────────────────────────────────────────────────────

    def _load_params(self) -> dict:
        names = [
            "conveyor_settle_s", "conveyor_advance_mm", "conveyor_speed_mms",
            "conveyor_accel_mms2", "conveyor_decel_mms2",
            "z_lower_pos_mm", "z_lower_speed_mms", "z_accel_mms2", "z_decel_mms2", "z_lower_wait_s",
            "peeler_position_angle_deg", "peeler_position_wait_s",
            "yuzu_rotation_rpm", "yuzu_rotation_accel_rpm_s", "yuzu_rotation_decel_rpm_s",
            "spin_up_wait_s",
            "peeler_advance_mm", "peeler_advance_speed_mms",
            "peeler_feed_accel_mms2", "peeler_feed_decel_mms2", "peeler_advance_wait_s",
            "peeler_orbit_rpm", "peeler_orbit_accel_rpm_s", "peeler_orbit_decel_rpm_s",
            "peel_duration_s",
            "grip_advance_mm", "grip_speed_mms", "grip_wait_s",
            "z_home_wait_s", "release_wait_s",
        ]
        return {n: self.get_parameter(n).value for n in names}

    def _send_linear(self, key: str, pos_mm: float, speed_mms: float,
                     accel_mms2: float = 1.0, decel_mms2: float = 1.0):
        msg = Float32MultiArray()
        msg.data = [
            float(pos_mm),
            float(speed_mms),
            float(accel_mms2),
            float(decel_mms2),
        ]
        self._pub[key].publish(msg)

    def _send_rpm(self, key: str, rpm: float,
                  accel_rpm_s: float = 0.0, decel_rpm_s: float = 0.0):
        """Publish a Type 2 spin command: [rpm, accel_rpm_s, effective_decel_rpm_s]."""
        effective_decel = decel_rpm_s if decel_rpm_s > 0.0 else accel_rpm_s
        msg = Float32MultiArray()
        msg.data = [float(rpm), float(accel_rpm_s), float(effective_decel)]
        self._pub[key].publish(msg)

    def _send_angle(self, key: str, angle_deg: float, speed_rpm: float,
                    accel_rpm_s: float = 0.0, decel_rpm_s: float = 0.0):
        """Publish a rotary position command: [angle_deg, speed_rpm, accel_rpm_s, decel_rpm_s].

        Used for Motor⑤ Step 3 (90° absolute position via ABZO encoder).
        Positive angle = CW; motor driver interprets relative to its home position.
        """
        effective_decel = decel_rpm_s if decel_rpm_s > 0.0 else accel_rpm_s
        msg = Float32MultiArray()
        msg.data = [float(angle_deg), float(speed_rpm),
                    float(accel_rpm_s), float(effective_decel)]
        self._pub[key].publish(msg)

    def _wait(self, duration_s: float) -> bool:
        """Timed wait; returns False immediately if abort fires."""
        aborted = self._abort_event.wait(timeout=duration_s)
        return not aborted

    def _wait_step(self, duration_s: float) -> bool:
        """Timed wait for step completion, then in manual mode pause for operator.

        Returns False if aborted at any point.
        """
        if not self._wait(duration_s):
            return False
        if self._manual_mode:
            self._step_event.clear()
            self._set_state("manual_pause")
            self.get_logger().info("Manual pause — waiting for /peel/next_step ...")
            # Poll so abort can interrupt the indefinite wait
            while not self._step_event.wait(timeout=0.2):
                if self._abort_event.is_set():
                    return False
            if self._abort_event.is_set():
                return False
        return True

    def _emergency_stop(self):
        self._pub["yuzu_stop"].publish(Empty())
        self._pub["orbit_stop"].publish(Empty())
        self._pub["orbit_home"].publish(Empty())
        self._pub["z_home"].publish(Empty())
        self._pub["peeler3_home"].publish(Empty())
        self._pub["peeler4_home"].publish(Empty())
        self._pub["conveyor_home"].publish(Empty())
        self._conveyor_pos_mm = 0.0
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
