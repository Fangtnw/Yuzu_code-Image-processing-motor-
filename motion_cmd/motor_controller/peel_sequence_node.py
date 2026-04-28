"""
Peel sequence node — orchestrates all 5 DOF for one yuzu peel cycle.

Machine spec (from 素案):
  Yuzu size      : long axis ~50 mm, short axis ~44 mm
  Cycle target   : 3 sec total (set 2 s + peel 1.7 s)
  Yuzu rotation  : 200–250 rpm
  Peeler orbit   : 20–30 rpm (10:1 gear), 0.5 rev = 1.5 s at 20 rpm
  Peeler tilt    : 21 degrees (fixed mechanical)

=============================================================
IMAGE TEAM INTERFACE
=============================================================
Step 1 — publish yuzu measurements (optional; improves depth adaptation):
  Topic : /yuzu_detected
  Type  : std_msgs/Float32MultiArray
  data  : [x_offset_mm, y_offset_mm, long_axis_mm, short_axis_mm]
    x_offset_mm   — lateral offset of yuzu centre from nominal (for future conveyor trim)
    y_offset_mm   — reserved
    long_axis_mm  — scales Z travel  (nominal 50 mm)
    short_axis_mm — scales peeler advance depth  (nominal 44 mm)

Step 2 — trigger the cycle:
  Topic : /peel/start
  Type  : std_msgs/Empty

Monitor:
  Topic : /peel/status
  Type  : std_msgs/String
  values: idle | feeding | lowering | spinning_up | peeling |
          retracting | spinning_down | raising | done | error | aborted

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

# Nominal yuzu dimensions (used to scale adaptive depths)
YUZU_LONG_NOM_MM  = 50.0
YUZU_SHORT_NOM_MM = 44.0


class PeelSequenceNode(Node):

    def __init__(self):
        super().__init__("peel_sequence")

        # ── Motion / timing parameters ──
        # All values tunable at runtime: ros2 param set /peel_sequence <name> <value>
        self.declare_parameter("conveyor_settle_s",          1.2)   # wait after conveyor step
        self.declare_parameter("z_lower_pos_mm",            25.0)   # how far Z descends onto yuzu
        self.declare_parameter("z_lower_speed_mms",         15.0)
        self.declare_parameter("z_lower_wait_s",             2.0)   # conservative wait for Z travel
        self.declare_parameter("yuzu_rotation_rpm",        225.0)   # midpoint of 200-250 rpm
        self.declare_parameter("spin_up_wait_s",             0.3)
        self.declare_parameter("peeler_orbit_rpm",          25.0)   # midpoint of 20-30 rpm
        self.declare_parameter("peeler_advance_mm",         10.0)   # blade push depth
        self.declare_parameter("peeler_advance_speed_mms",   8.0)
        self.declare_parameter("peel_duration_s",            1.5)   # 0.5 orbit at 20 rpm = 1.5 s
        self.declare_parameter("retract_wait_s",             0.8)
        self.declare_parameter("spin_down_wait_s",           0.3)
        self.declare_parameter("z_raise_speed_mms",         20.0)
        self.declare_parameter("z_raise_wait_s",             2.0)

        # ── Publishers to all 5 motors ──
        self._pub = {
            # Motor 1: Z-axis (DR28T linear)
            "z_cmd":             self.create_publisher(Float32MultiArray, "/motor_z/motor_cmd",  10),
            "z_home":            self.create_publisher(Empty,             "/motor_z/motor_home", 10),
            # Motor 2: Yuzu rotation spindle
            "yuzu_spin":         self.create_publisher(Float32,           "/motor_yuzu_rot/motor_spin", 10),
            "yuzu_stop":         self.create_publisher(Empty,             "/motor_yuzu_rot/motor_stop", 10),
            # Motor 3: Peeler orbit arm
            "orbit_spin":        self.create_publisher(Float32,           "/motor_peeler_orbit/motor_spin", 10),
            "orbit_stop":        self.create_publisher(Empty,             "/motor_peeler_orbit/motor_stop", 10),
            # Motor 4: Peeler radial advance (DR28T linear)
            "advance_cmd":       self.create_publisher(Float32MultiArray, "/motor_peeler_advance/motor_cmd",  10),
            "advance_home":      self.create_publisher(Empty,             "/motor_peeler_advance/motor_home", 10),
            # Motor 5: Conveyor
            "conveyor_step":     self.create_publisher(Empty,             "/motor_conveyor/motor_step", 10),
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
        self.get_logger().info("Peel sequence node ready.")
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

            # ── Adapt depths to detected yuzu size ──
            z_lower = p["z_lower_pos_mm"]
            blade_advance = p["peeler_advance_mm"]
            if self._yuzu_info:
                z_lower       = min(29.0, z_lower * (self._yuzu_info["long_axis_mm"]  / YUZU_LONG_NOM_MM))
                blade_advance = min(29.0, blade_advance * (self._yuzu_info["short_axis_mm"] / YUZU_SHORT_NOM_MM))
                self.get_logger().info(
                    f"Adapted — z_lower={z_lower:.1f} mm  blade_advance={blade_advance:.1f} mm"
                )

            # 1. Advance conveyor one tray
            self._set_state("feeding")
            self._pub["conveyor_step"].publish(Empty())
            if not self._wait(p["conveyor_settle_s"]): return

            # 2. Lower Z-axis spindle onto yuzu
            self._set_state("lowering")
            self._send_linear("z_cmd", z_lower, p["z_lower_speed_mms"])
            if not self._wait(p["z_lower_wait_s"]): return

            # 3. Spin up yuzu rotation
            self._set_state("spinning_up")
            self._send_rpm("yuzu_spin", p["yuzu_rotation_rpm"])
            if not self._wait(p["spin_up_wait_s"]): return

            # 4. Peel: start peeler orbit + advance blade simultaneously
            self._set_state("peeling")
            self._send_rpm("orbit_spin", p["peeler_orbit_rpm"])
            self._send_linear("advance_cmd", blade_advance, p["peeler_advance_speed_mms"])
            if not self._wait(p["peel_duration_s"]): return

            # 5. Retract blade + stop peeler orbit
            self._set_state("retracting")
            self._send_linear("advance_cmd", 0.0, p["peeler_advance_speed_mms"] * 2.0)
            self._pub["orbit_stop"].publish(Empty())
            if not self._wait(p["retract_wait_s"]): return

            # 6. Stop yuzu rotation
            self._set_state("spinning_down")
            self._pub["yuzu_stop"].publish(Empty())
            if not self._wait(p["spin_down_wait_s"]): return

            # 7. Raise Z-axis
            self._set_state("raising")
            self._send_linear("z_cmd", 0.0, p["z_raise_speed_mms"])
            if not self._wait(p["z_raise_wait_s"]): return

            self._set_state("done")

        except Exception as exc:
            self.get_logger().error(f"Sequence exception: {exc}")
            self._emergency_stop()
            self._set_state("error")

    # ── Helpers ────────────────────────────────────────────────────

    def _load_params(self) -> dict:
        names = [
            "conveyor_settle_s", "z_lower_pos_mm", "z_lower_speed_mms", "z_lower_wait_s",
            "yuzu_rotation_rpm", "spin_up_wait_s",
            "peeler_orbit_rpm", "peeler_advance_mm", "peeler_advance_speed_mms", "peel_duration_s",
            "retract_wait_s", "spin_down_wait_s",
            "z_raise_speed_mms", "z_raise_wait_s",
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
        """Block for duration_s, or return False early if abort received."""
        aborted = self._abort_event.wait(timeout=duration_s)
        return not aborted

    def _emergency_stop(self):
        self._pub["yuzu_stop"].publish(Empty())
        self._pub["orbit_stop"].publish(Empty())
        self._pub["z_home"].publish(Empty())
        self._pub["advance_home"].publish(Empty())
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
