"""Feedback-relative, tightly bounded commissioning guard for Motor 3."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


PUBLIC_TOPIC = "/motor3_commissioning/offset_mm"
RAW_TOPIC = "/motor3_raw_position_controller/commands"
JOINT_NAME = "motor3_joint"
MAX_OFFSET_MM = 2.000
MIN_INCREMENT_MM = 0.001
MAX_VELOCITY_M_S = 0.008
MAX_ACCELERATION_M_S2 = 0.050
UPDATE_RATE_HZ = 200.0


class Motor3CommissioningGuard(Node):
    def __init__(self) -> None:
        super().__init__("azd3a_motor3_commissioning_guard")
        self.origin = None
        self.command_position = None
        self.target_position = None
        self.command_velocity = 0.0
        self.operator_command_received = False
        self.raw_publisher = self.create_publisher(Float64MultiArray, RAW_TOPIC, 10)
        self.create_subscription(Float64, PUBLIC_TOPIC, self.on_command, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_timer(1.0 / UPDATE_RATE_HZ, self.update_command)
        self.get_logger().info(
            "Motor 3 commissioning guard ready: relative -2.000..2.000 mm, "
            "0.001 mm increments, 8.0 mm/s, 50 mm/s^2 ramp"
        )

    def on_joint_state(self, message: JointState) -> None:
        try:
            position = message.position[message.name.index(JOINT_NAME)]
        except (ValueError, IndexError):
            return
        if not math.isfinite(position):
            return
        if not self.operator_command_received:
            self.origin = position
            self.command_position = position
            self.target_position = position
            self.command_velocity = 0.0

    def on_command(self, message: Float64) -> None:
        offset_mm = message.data
        if not math.isfinite(offset_mm):
            self.get_logger().error("REJECTED Motor 3 offset: expected a finite value")
            return
        if not -MAX_OFFSET_MM <= offset_mm <= MAX_OFFSET_MM:
            self.get_logger().error("REJECTED Motor 3 offset: permitted range is -2.000..2.000 mm")
            return
        increments = round(offset_mm / MIN_INCREMENT_MM)
        if abs(offset_mm - increments * MIN_INCREMENT_MM) > 1e-9:
            self.get_logger().error("REJECTED Motor 3 offset: use 0.001 mm increments")
            return
        if self.origin is None or self.command_position is None:
            self.get_logger().error("REJECTED Motor 3 offset: no valid feedback yet")
            return
        self.operator_command_received = True
        self.target_position = self.origin + offset_mm / 1000.0
        self.get_logger().info(f"Accepted Motor 3 startup-relative offset {offset_mm:.3f} mm")

    def publish_raw(self) -> None:
        message = Float64MultiArray()
        message.data = [self.command_position]
        self.raw_publisher.publish(message)

    def update_command(self) -> None:
        if self.command_position is None or self.target_position is None:
            return
        dt = 1.0 / UPDATE_RATE_HZ
        error = self.target_position - self.command_position
        if abs(error) < 1e-12 and abs(self.command_velocity) < 1e-12:
            self.command_position = self.target_position
            self.command_velocity = 0.0
            self.publish_raw()
            return
        direction = 1.0 if error >= 0.0 else -1.0
        stopping_distance = self.command_velocity**2 / (2.0 * MAX_ACCELERATION_M_S2)
        if self.command_velocity * direction < 0.0:
            acceleration = direction * MAX_ACCELERATION_M_S2
        elif abs(error) <= stopping_distance:
            acceleration = -math.copysign(MAX_ACCELERATION_M_S2, self.command_velocity)
        else:
            acceleration = direction * MAX_ACCELERATION_M_S2
        velocity = self.command_velocity + acceleration * dt
        velocity = max(-MAX_VELOCITY_M_S, min(MAX_VELOCITY_M_S, velocity))
        position = self.command_position + velocity * dt
        if direction * (self.target_position - position) <= 0.0:
            position = self.target_position
            velocity = 0.0
        self.command_position = position
        self.command_velocity = velocity
        self.publish_raw()


def main() -> None:
    rclpy.init()
    node = Motor3CommissioningGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
