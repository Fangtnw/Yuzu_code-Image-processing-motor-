"""Coordinate guarded Motors 3, 4, and 5 through one frozen contour."""

import argparse
import json
import math
import time

from .yuzu_step3_plan import build_step3_actions, interpolated_feed


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--execute", action="store_true", help="Send motor targets; default only previews")
    p.add_argument("--motor3-center-mm", type=float, default=35.0)
    p.add_argument("--motor4-center-mm", type=float, default=35.0)
    p.add_argument("--source-inset-mm", type=float, default=3.0)
    p.add_argument("--peel-depth-mm", type=float, default=0.0)
    p.add_argument("--contour-timeout-s", type=float, default=10.0)
    p.add_argument("--contour-file", help="JSON file containing one previously captured 72-ray contour")
    p.add_argument("--waypoint-timeout-s", type=float, default=15.0)
    p.add_argument("--sweep-speed-deg-s", type=float, default=10.0,
                   help="Continuous Motor 5 peel speed (1..30 deg/s)")
    return p


def main():
    args = parser().parse_args()
    if args.contour_timeout_s <= 0 or args.waypoint_timeout_s <= 0:
        raise SystemExit("Timeouts must be positive")
    if not 1.0 <= args.sweep_speed_deg_s <= 30.0:
        raise SystemExit("--sweep-speed-deg-s must be within 1..30")

    import rclpy
    from control_msgs.msg import DynamicJointState
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32MultiArray, Float64

    class Coordinator(Node):
        def __init__(self):
            super().__init__("yuzu_step3_sequence")
            self.contour = None
            self.zero_rad = None
            self.positions = {}
            self.statuses = {}
            self.feedback_at = 0.0
            self.motor4_at = 0.0
            self.create_subscription(Float32MultiArray, "/yuzu/contour_point_offsets", self.on_contour, 10)
            self.create_subscription(Float64, "/yuzu/motor5/zero_position_rad", self.on_zero,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(DynamicJointState, "/dynamic_joint_states", self.on_feedback, 10)
            self.create_subscription(JointState, "/joint_states", self.on_joint, 10)
            self.pubs = {motor: self.create_publisher(Float64, topic, 10) for motor, topic in (
                (3, "/yuzu/motor3/target_position_mm"),
                (4, "/yuzu/motor4/target_position_mm"),
                (5, "/yuzu/motor5/target_position_deg"))}
            self.speed_pub = self.create_publisher(Float64, "/yuzu/motor5/max_velocity_deg_s", 10)

        def on_contour(self, msg):
            if self.contour is None:
                self.contour = tuple(msg.data)

        def on_zero(self, msg):
            if math.isfinite(msg.data):
                self.zero_rad = msg.data

        def on_feedback(self, msg):
            values = {}
            statuses = {}
            for joint, interfaces in zip(msg.joint_names, msg.interface_values):
                for name, value in zip(interfaces.interface_names, interfaces.values):
                    if joint == "motor4_joint" and name == "motor3_position":
                        values[3] = value * 1000
                    elif joint == "motor4_joint" and name == "motor5_position":
                        values[5] = value
                    elif joint == "motor4_joint" and name in ("motor3_status", "motor4_status", "motor5_status"):
                        statuses[int(name[5])] = int(value)
            if len(values) == 2 and len(statuses) == 3 and all(math.isfinite(v) for v in values.values()):
                self.positions.update(values)
                self.statuses = statuses
                self.feedback_at = time.monotonic()

        def on_joint(self, msg):
            try:
                value = msg.position[msg.name.index("motor4_joint")]
            except (ValueError, IndexError):
                return
            if math.isfinite(value):
                self.positions[4] = value * 1000
                self.motor4_at = time.monotonic()

        def feedback_ok(self):
            return (time.monotonic() - self.feedback_at < 0.5
                    and time.monotonic() - self.motor4_at < 0.5
                    and len(self.positions) == 3 and len(self.statuses) == 3
                    and all((status & 0x006F) == 0x0027 for status in self.statuses.values())
                    and self.zero_rad is not None)

        def relative_angle(self):
            return math.degrees(self.positions[5] - self.zero_rad)

        def at_target(self, action):
            return (abs(self.positions[3] - action.motor3_mm) <= 0.05
                    and abs(self.positions[4] - action.motor4_mm) <= 0.05
                    and abs(self.relative_angle() - action.motor5_deg) <= 0.1)

    rclpy.init(args=[])
    node = Coordinator()
    motion_started = False
    try:
        if args.contour_file:
            with open(args.contour_file, encoding="utf-8") as handle:
                node.contour = tuple(json.load(handle))
        else:
            deadline = time.monotonic() + args.contour_timeout_s
            while node.contour is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
        if node.contour is None:
            raise RuntimeError("No /yuzu/contour_point_offsets received")
        actions = build_step3_actions(node.contour, args.motor3_center_mm,
                                      args.motor4_center_mm, args.source_inset_mm,
                                      args.peel_depth_mm)
        print(f"Frozen 72-ray contour; {len(actions)} actions; outer-surface feed depth "
              f"{args.peel_depth_mm:g} mm", flush=True)
        for action in actions:
            print(f"{action.phase:16} M5={action.motor5_deg:+6.1f} deg "
                  f"M3={action.motor3_mm:7.4f} mm M4={action.motor4_mm:7.4f} mm", flush=True)
        if not args.execute:
            print("Preview only; no motor commands sent.", flush=True)
            return

        deadline = time.monotonic() + args.contour_timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (node.feedback_ok() and all(pub.get_subscription_count() > 0 for pub in node.pubs.values())
                    and node.speed_pub.get_subscription_count() > 0):
                break
        else:
            raise RuntimeError("Missing fresh enabled motor feedback, runtime zero, or guarded command/speed subscriber; restart the Motor 5 guard with updated code")
        if (abs(node.positions[3]) > 0.05 or abs(node.positions[4]) > 0.05
                or abs(node.relative_angle()) > 0.1):
            raise RuntimeError(f"Start requires M3/M4 at 0 mm and M5 at runtime zero; got "
                               f"{node.positions[3]:.3f}, {node.positions[4]:.3f} mm, "
                               f"{node.relative_angle():.3f} deg")
        def wait_for_action(index, action):
            deadline = time.monotonic() + args.waypoint_timeout_s
            stable_at = None
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if not node.feedback_ok():
                    raise RuntimeError(f"Feedback stale or drive disabled at action {index}; sequence stopped")
                if node.at_target(action):
                    stable_at = stable_at or time.monotonic()
                    if time.monotonic() - stable_at >= 0.3:
                        return
                else:
                    stable_at = None
            raise RuntimeError(f"Action {index} ({action.phase}) did not settle; sequence stopped")

        for index, action in enumerate(actions[:2], 1):
            if not node.feedback_ok():
                raise RuntimeError("Drive feedback stale or drive not enabled; sequence stopped")
            for motor in action.command_motors:
                target = {3: action.motor3_mm, 4: action.motor4_mm,
                          5: action.motor5_deg}[motor]
                node.pubs[motor].publish(Float64(data=float(target)))
            motion_started = True
            wait_for_action(index, action)
            print(f"Completed {index}/{len(actions)}: {action.phase} M5 {action.motor5_deg:+g} deg", flush=True)

        # One continuous M5 target. Follow measured angle with interpolated slide
        # positions; the 0.3° look-ahead compensates for guard and ROS latency.
        node.speed_pub.publish(Float64(data=args.sweep_speed_deg_s))
        ready_at = time.monotonic() + 0.25
        while time.monotonic() < ready_at:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.feedback_ok():
            raise RuntimeError("Feedback stale before continuous sweep")
        node.pubs[5].publish(Float64(data=-10.0))
        sweep_end = actions[-3]
        sweep_started_at = time.monotonic()
        deadline = time.monotonic() + max(args.waypoint_timeout_s,
                                          100.0 / args.sweep_speed_deg_s + 8.0)
        stable_at = None
        last_send = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if not node.feedback_ok():
                raise RuntimeError("Feedback stale or drive disabled during continuous sweep")
            angle = node.relative_angle()
            if angle < -10.5 or angle > 90.5:
                raise RuntimeError(f"Motor 5 left sweep bounds at {angle:.2f} deg")
            desired = interpolated_feed(actions, max(-10.0, min(90.0, angle - 0.3)))
            now = time.monotonic()
            if (now - sweep_started_at > 0.5 and
                    (abs(node.positions[3] - desired[0]) > 1.0 or
                     abs(node.positions[4] - desired[1]) > 1.0)):
                raise RuntimeError("A slide fell more than 1 mm behind the contour; sweep stopped")
            if now - last_send >= 0.05:
                node.pubs[3].publish(Float64(data=desired[0]))
                node.pubs[4].publish(Float64(data=desired[1]))
                last_send = now
            if abs(angle + 10.0) <= 0.1:
                # At the endpoint command exact final feed targets and wait.
                node.pubs[3].publish(Float64(data=sweep_end.motor3_mm))
                node.pubs[4].publish(Float64(data=sweep_end.motor4_mm))
                if node.at_target(sweep_end):
                    stable_at = stable_at or now
                    if now - stable_at >= 0.3:
                        break
                else:
                    stable_at = None
        else:
            raise RuntimeError("Continuous sweep did not settle; sequence stopped")
        print("Completed continuous +90 to -10 deg contour sweep", flush=True)

        for index, action in enumerate(actions[-2:], len(actions) - 1):
            if not node.feedback_ok():
                raise RuntimeError("Drive feedback stale before release/return")
            for motor in action.command_motors:
                target = {3: action.motor3_mm, 4: action.motor4_mm,
                          5: action.motor5_deg}[motor]
                node.pubs[motor].publish(Float64(data=float(target)))
            wait_for_action(index, action)
            print(f"Completed {index}/{len(actions)}: {action.phase}", flush=True)
        node.speed_pub.publish(Float64(data=900.0))
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.02)
        print("Step 3 complete: slides at 0 mm, Motor 5 at origin.", flush=True)
    except (RuntimeError, ValueError, KeyboardInterrupt) as error:
        if motion_started and node.feedback_ok():
            node.pubs[5].publish(Float64(data=max(-180.0, min(180.0, node.relative_angle()))))
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.02)
        raise SystemExit(f"Step 3 stopped: {error}") from error
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
