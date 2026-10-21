"""Read the existing detector topic and publish a non-motion profile preview."""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .yuzu_contour_profile import ContourProfile


class ContourPreview(Node):
    def __init__(self):
        super().__init__("yuzu_contour_preview")
        for key, value in (("side_image_angle_deg", 90.0), ("angle_direction", 1),
                           ("zero_radius_mm", 35.0), ("source_inset_mm", 3.0),
                           ("peel_depth_mm", 0.0), ("motor3_offset_mm", 0.0),
                           ("motor4_offset_mm", 0.0)):
            self.declare_parameter(key, value)
        self.last_received = None
        self.pub = self.create_publisher(String, "/yuzu/contour_preview", 1)
        self.create_subscription(Float32MultiArray, "/yuzu/yellow_dot_offsets", self.receive, 1)
        self.create_timer(0.5, self.check_stale)

    def emit(self, payload):
        self.pub.publish(String(data=json.dumps(payload, allow_nan=False)))

    def check_stale(self):
        if self.last_received is None or time.monotonic() - self.last_received > 1.0:
            self.emit({"valid": False, "reason": "No fresh contour within 1 second"})

    def receive(self, msg):
        self.last_received = time.monotonic()
        try:
            profile = ContourProfile(msg.data)
            p = lambda key: self.get_parameter(key).value
            rows = []
            # Measured 30-degree knots plus interpolated sweep endpoints.
            for angle in (-45, -30, 0, 30, 45):
                feeds = [profile.feed_at(angle, p("side_image_angle_deg"), p("angle_direction"),
                         p("zero_radius_mm"), p("source_inset_mm"), p("peel_depth_mm"),
                         p(f"motor{motor}_offset_mm")) for motor in (3, 4)]
                rows.append({"motor5_deg": angle, "motor3_mm": feeds[0], "motor4_mm": feeds[1]})
            self.emit({"valid": True, "calibration": "provisional", "motion_enabled": False,
                       "source_spacing_deg": 30, "targets": rows})
        except (ValueError, TypeError, OverflowError) as error:
            self.emit({"valid": False, "reason": str(error)})


def main():
    rclpy.init()
    node = ContourPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
