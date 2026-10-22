"""Publish the camera's horizontal diameter as a no-motion JSON preview."""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .yuzu_contour_profile import ContourProfile


class ContourPreview(Node):
    def __init__(self):
        super().__init__("yuzu_contour_preview")
        self.declare_parameter("source_inset_mm", 3.0)
        self.last_received = None
        self.pub = self.create_publisher(String, "/yuzu/contour_preview", 1)
        self.create_subscription(Float32MultiArray, "/yuzu/yellow_dot_offsets", self.receive, 1)
        self.create_timer(0.5, self.check_stale)

    def emit(self, payload):
        self.pub.publish(String(data=json.dumps(payload, allow_nan=False)))

    def check_stale(self):
        if self.last_received is None or time.monotonic() - self.last_received > 1.0:
            self.emit({"valid": False, "reason": "No fresh detector message within 1 second"})

    def receive(self, msg):
        self.last_received = time.monotonic()
        try:
            measurement = ContourProfile(msg.data)
            inset = float(self.get_parameter("source_inset_mm").value)
            self.emit({
                "valid": True,
                "measurement": "horizontal_diameter",
                "horizontal_diameter_mm": measurement.horizontal_diameter_mm(inset),
                "source_inset_mm": inset,
                "motion_enabled": False,
            })
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
