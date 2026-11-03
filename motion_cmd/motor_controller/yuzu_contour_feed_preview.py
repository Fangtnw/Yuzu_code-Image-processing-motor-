"""Print no-motion Motor 3/4 feed targets for one complete camera contour."""

import argparse
import time

from .yuzu_contour_feed import ContourFeedProfile
from .yuzu_motor5_orbit_plan import orbit_waypoints


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor3-center-mm", type=float, default=35.0,
                        help="Motor 3 blade distance from fruit center at slide 0 mm")
    parser.add_argument("--motor4-center-mm", type=float, default=35.0,
                        help="Motor 4 blade distance from fruit center at slide 0 mm")
    parser.add_argument("--timeout-s", type=float, default=5.0)
    return parser


def build_rows(data, motor3_center_mm=35.0, motor4_center_mm=35.0):
    profile = ContourFeedProfile(data)
    return tuple(
        profile.targets_at_motor5(
            waypoint.motor5_target_deg,
            motor3_center_distance_mm=motor3_center_mm,
            motor4_center_distance_mm=motor4_center_mm,
        )
        for waypoint in orbit_waypoints()
        if waypoint.update_slides
    )


def main():
    args = build_parser().parse_args()
    if not 0.0 < args.timeout_s <= 60.0:
        raise SystemExit("--timeout-s must be within 0..60 seconds")

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray

    class ContourReader(Node):
        def __init__(self):
            super().__init__("yuzu_contour_feed_preview_reader")
            self.data = None
            self.create_subscription(Float32MultiArray, "/yuzu/contour_point_offsets",
                                     self.receive, 10)

        def receive(self, message):
            self.data = list(message.data)

    rclpy.init(args=[])
    node = ContourReader()
    try:
        deadline = time.monotonic() + args.timeout_s
        while node.data is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.data is None:
            raise RuntimeError("No complete /yuzu/contour_point_offsets message received")
        rows = build_rows(node.data, args.motor3_center_mm, args.motor4_center_mm)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print("No motion commands. Motor 5 sweep, mirror-averaged slide targets (mm):")
    print("M5 deg | M3 rays deg | M3 mm | M4 rays deg | M4 mm")
    for row in rows:
        print(f"{row['motor5_deg']:6.1f} | "
              f"{row['motor3_contour_angle_deg']:5.1f}/{row['motor3_mirror_angle_deg']:5.1f} | "
              f"{row['motor3_target_mm']:6.4f} | "
              f"{row['motor4_contour_angle_deg']:5.1f}/{row['motor4_mirror_angle_deg']:5.1f} | "
              f"{row['motor4_target_mm']:6.4f}")


if __name__ == "__main__":
    main()
