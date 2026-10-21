"""Synthetic detector output with the same radial inset as the camera node."""

import math


def yellow_points(horizontal_radius_mm=30.0, vertical_radius_mm=27.0,
                  source_inset_mm=3.0):
    """Sample an ellipse along the detector's 11 yellow rays (units: mm)."""
    values = (horizontal_radius_mm, vertical_radius_mm, source_inset_mm)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Mock dimensions must be finite")
    if min(horizontal_radius_mm, vertical_radius_mm) <= 0:
        raise ValueError("Ellipse radii must be positive")
    if not 0 <= source_inset_mm < min(horizontal_radius_mm, vertical_radius_mm):
        raise ValueError("Inset must be nonnegative and smaller than both radii")
    data = []
    for angle in range(30, 360, 30):
        theta = math.radians(angle)
        sx, sy = math.sin(theta), math.cos(theta)
        radius = 1 / math.sqrt((sx / horizontal_radius_mm)**2 +
                               (sy / vertical_radius_mm)**2)
        radius -= source_inset_mm
        data.extend((float(angle), radius * sx, radius * sy))
    return data


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, MultiArrayDimension

    class MockContour(Node):
        def __init__(self):
            super().__init__("yuzu_mock_contour")
            for name, default in (("horizontal_radius_mm", 30.0),
                                  ("vertical_radius_mm", 27.0),
                                  ("source_inset_mm", 3.0), ("publish_rate_hz", 10.0)):
                self.declare_parameter(name, default)
            p = lambda name: self.get_parameter(name).value
            rate = p("publish_rate_hz")
            if not math.isfinite(rate) or not 2 <= rate <= 100:
                raise ValueError("publish_rate_hz must be 2..100 (preview timeout is 1 s)")
            self.message = Float32MultiArray()
            self.message.data = yellow_points(p("horizontal_radius_mm"),
                                             p("vertical_radius_mm"), p("source_inset_mm"))
            self.message.layout.dim = [
                MultiArrayDimension(label="yellow_dots", size=11, stride=33),
                MultiArrayDimension(label="angle_dx_dy_mm", size=3, stride=3)]
            self.publisher = self.create_publisher(Float32MultiArray, "/yuzu/yellow_dot_offsets", 1)
            self.create_timer(1 / rate, self.publish)
            self.get_logger().info("MOCK detector active; stop the real detector first. No motor commands published.")

        def publish(self):
            self.publisher.publish(self.message)

    rclpy.init(args=args)
    node = None
    try:
        node = MockContour()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
