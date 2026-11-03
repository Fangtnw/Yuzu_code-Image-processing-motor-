"""Preview or command Motor 1 using the camera's top-point Y offset."""

import argparse
import math
import time

from .yuzu_motor1_reference import Motor1CameraReference


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    offset = parser.add_mutually_exclusive_group(required=True)
    offset.add_argument("--y-offset-mm", type=float,
                        help="Signed red-point offset from frame center in millimetres")
    offset.add_argument("--offset-from-detector", action="store_true",
                        help="Wait for a fresh /yuzu/y_offset_mm Float32 value")
    parser.add_argument("--frame-center-mm", type=float, default=204.0,
                        help="Motor 1 encoder position when holder tip reaches frame center; default 204 mm")
    parser.add_argument("--home-mm", type=float, default=0.0,
                        help="Motor 1 encoder position at the measured home pose")
    parser.add_argument("--pierce-depth-mm", type=float, default=3.0,
                        help="Desired depth beneath the original outer yuzu top")
    parser.add_argument("--detector-inset-mm", type=float, default=3.0,
                        help="Depth of the detector's red point beneath the outer top")
    parser.add_argument("--allowance-mm", type=float, default=0.0,
                        help="Additional downward insertion beyond the pierce depth; default 0 mm")
    parser.add_argument("--offset-multiplier", type=int, choices=(-1, 1), default=-1,
                        help="Motor travel sign for a positive image Y offset; default -1")
    parser.add_argument("--lift-mm", type=float, default=None,
                        help="Step 2 lift distance, 100..150 mm; required to execute lift")
    parser.add_argument("--step", choices=("plan", "insert", "lift", "home", "demo"), default="plan",
                        help="demo performs insertion then lift using one captured offset")
    parser.add_argument("--execute", action="store_true",
                        help="Publish the selected guarded Motor 1 target")
    return parser


def main():
    args = build_parser().parse_args()
    if args.offset_from_detector:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32

        class OffsetReader(Node):
            def __init__(self):
                super().__init__("yuzu_top_offset_reader")
                self.offset = None
                self.create_subscription(Float32, "/yuzu/y_offset_mm",
                                         self.receive, 10)

            def receive(self, msg):
                if math.isfinite(msg.data):
                    self.offset = msg.data

        rclpy.init(args=[])
        reader = OffsetReader()
        try:
            deadline = time.monotonic() + 5.0
            while reader.offset is None and time.monotonic() < deadline:
                rclpy.spin_once(reader, timeout_sec=0.1)
            if reader.offset is None:
                raise RuntimeError("No fresh /yuzu/y_offset_mm value within 5 seconds")
            args.y_offset_mm = reader.offset
        finally:
            reader.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    if args.execute and args.step in ("lift", "demo") and args.lift_mm is None:
        raise SystemExit("Specify --lift-mm 100..150 explicitly for Step 2 motion")
    lift_mm = 120.0 if args.lift_mm is None else args.lift_mm
    targets = Motor1CameraReference(
        args.frame_center_mm, args.home_mm, args.pierce_depth_mm,
        args.detector_inset_mm, args.allowance_mm, args.offset_multiplier,
    ).targets(args.y_offset_mm, lift_mm)
    print(f"Camera Y offset {targets['y_offset_mm']:+.4f} mm; "
          f"home {targets['home_position_mm']:.4f} mm; "
          f"frame-center encoder {targets['frame_center_position_mm']:.4f} mm; "
          f"pierce depth {targets['pierce_depth_mm']:.4f} mm; "
          f"detector inset {targets['detector_inset_mm']:.4f} mm; "
          f"extra allowance {targets['allowance_mm']:.4f} mm")
    print(f"Motor 1 target, positive downward: Step 1 insert {targets['insertion_target_mm']:.4f} mm; "
          f"Step 2 lift {targets['lift_target_mm']:.4f} mm")
    if not args.execute:
        return
    if args.step == "plan":
        raise SystemExit("Choose --step insert, lift, or demo with --execute")

    import rclpy
    from control_msgs.msg import DynamicJointState
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64

    class Motor1Client(Node):
        def __init__(self):
            super().__init__("yuzu_motor1_reference_client")
            self.position_mm = None
            self.position_time = 0.0
            self.enabled = False
            self.status_time = 0.0
            self.pub = self.create_publisher(Float64, "/yuzu/motor1/target_position_mm", 10)
            self.create_subscription(JointState, "/joint_states", self.on_joint, 10)
            self.create_subscription(DynamicJointState, "/dynamic_joint_states", self.on_dynamic, 10)

        def on_joint(self, msg):
            try:
                position = msg.position[msg.name.index("motor1_motor2")]
            except (ValueError, IndexError):
                return
            if math.isfinite(position):
                self.position_mm = position * 1000.0
                self.position_time = time.monotonic()

        def on_dynamic(self, msg):
            try:
                joint = msg.interface_values[msg.joint_names.index("motor1_motor2")]
                status = joint.values[joint.interface_names.index("motor1_status")]
            except (ValueError, IndexError):
                return
            if math.isfinite(status):
                self.enabled = (int(status) & 0x006F) == 0x0027
                self.status_time = time.monotonic()

        def ready(self):
            now = time.monotonic()
            return (self.position_mm is not None and self.enabled and
                    now - self.position_time < 0.5 and now - self.status_time < 0.5)

    rclpy.init(args=[])
    node = Motor1Client()
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.ready():
            raise RuntimeError("Motor 1 has no fresh position and Operation Enabled feedback")
        if args.step in ("insert", "demo") and abs(node.position_mm - targets["home_position_mm"]) > 0.1:
            raise RuntimeError("Step 1 insertion requires Motor 1 at the measured home position")
        if args.step == "lift" and abs(node.position_mm - targets["insertion_target_mm"]) > 0.1:
            raise RuntimeError("Step 2 lift requires Motor 1 at the calculated Step 1 insertion target")
        subscriber_deadline = time.monotonic() + 5.0
        while node.pub.get_subscription_count() == 0 and time.monotonic() < subscriber_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.pub.get_subscription_count() == 0:
            raise RuntimeError("No subscriber on guarded Motor 1 command topic within 5 seconds")
        if not node.ready():
            raise RuntimeError("Motor 1 feedback/status became stale or disabled before command")

        def publish_and_wait(stage, selected):
            node.pub.publish(Float64(data=selected))
            print(f"Published guarded Motor 1 {stage} target {selected:.4f} mm", flush=True)
            deadline = time.monotonic() + 30.0
            settled_since = None
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if not node.ready():
                    raise RuntimeError("Motor 1 feedback/status became stale or disabled; inspect drive and stop machine")
                if abs(node.position_mm - selected) <= 0.05:
                    settled_since = settled_since or time.monotonic()
                    if time.monotonic() - settled_since >= 0.3:
                        print(f"Motor 1 reached {node.position_mm:.4f} mm", flush=True)
                        return
                else:
                    settled_since = None
            raise RuntimeError(f"Motor 1 did not reach {stage} target within 30 seconds")

        if args.step in ("insert", "demo"):
            publish_and_wait("insert", targets["insertion_target_mm"])
        if args.step == "demo":
            if abs(node.position_mm - targets["insertion_target_mm"]) > 0.1:
                raise RuntimeError("Step 2 requires Motor 1 at the calculated insertion target")
            publish_and_wait("lift", targets["lift_target_mm"])
            print("Motor 1 Step 1 and Step 2 demo complete", flush=True)
        elif args.step == "lift":
            publish_and_wait("lift", targets["lift_target_mm"])
        elif args.step == "home":
            publish_and_wait("home", targets["home_position_mm"])
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
