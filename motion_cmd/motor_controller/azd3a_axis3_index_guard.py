"""Feedback-synchronized, bounded Axis 3 relative-angle command guard."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


UPDATE_RATE_HZ = 200.0
HARD_MAX_RPM = 20.0
HARD_MAX_ACCELERATION_RPM_S = 60.0
PUBLIC_TOPIC = "/axis3_position_controller/commands_deg"
RAW_TOPIC = "/axis3_raw_position_controller/commands"


class Axis3IndexGuard(Node):
    def __init__(self) -> None:
        super().__init__("azd3a_axis3_index_guard")
        self.max_rpm = float(self.declare_parameter("max_rpm", 5.0).value)
        self.max_acceleration_rpm_s = float(
            self.declare_parameter("max_acceleration_rpm_s", 5.0).value
        )
        if not 0.0 < self.max_rpm <= HARD_MAX_RPM:
            raise ValueError("max_rpm must be within (0, 20]")
        if not 0.0 < self.max_acceleration_rpm_s <= HARD_MAX_ACCELERATION_RPM_S:
            raise ValueError("max_acceleration_rpm_s must be within (0, 60]")

        self.measured_position = None
        self.origin = None
        self.command_position = None
        self.target_position = None
        self.command_velocity = 0.0
        self.operator_command_received = False
        self.publisher = self.create_publisher(Float64MultiArray, RAW_TOPIC, 10)
        self.create_subscription(Float64MultiArray, PUBLIC_TOPIC, self.on_command, 10)
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)
        self.create_timer(1.0 / UPDATE_RATE_HZ, self.update)
        self.get_logger().info(
            f"Axis 3 index guard ready: 0..90 deg CW, {self.max_rpm:.1f} rpm max, "
            f"{self.max_acceleration_rpm_s:.1f} rpm/s acceleration"
        )

    def on_state(self, message: JointState) -> None:
        try:
            position = message.position[message.name.index("axis3_joint")]
        except (ValueError, IndexError):
            return
        if not math.isfinite(position):
            return
        self.measured_position = position
        if not self.operator_command_received:
            self.origin = position
            self.command_position = position
            self.target_position = position
            self.command_velocity = 0.0

    def on_command(self, message: Float64MultiArray) -> None:
        if len(message.data) != 1 or not math.isfinite(message.data[0]):
            self.get_logger().error("REJECTED: expected one finite degree offset")
            return
        offset_deg = float(message.data[0])
        if not 0.0 <= offset_deg <= 90.0:
            self.get_logger().error("REJECTED: offset must be within 0..90 degrees CW")
            return
        if self.origin is None or self.command_position is None:
            self.get_logger().error("REJECTED: no valid Axis 3 feedback yet")
            return
        self.operator_command_received = True
        self.target_position = self.origin + math.radians(offset_deg)
        self.get_logger().info(f"Accepted Axis 3 index target {offset_deg:.3f} deg CW")

    def publish(self) -> None:
        message = Float64MultiArray()
        message.data = [self.command_position]
        self.publisher.publish(message)

    def update(self) -> None:
        if self.command_position is None or self.target_position is None:
            return
        dt = 1.0 / UPDATE_RATE_HZ
        error = self.target_position - self.command_position
        max_velocity = self.max_rpm * math.pi / 30.0
        max_acceleration = self.max_acceleration_rpm_s * math.pi / 30.0
        if abs(error) < 1e-10 and abs(self.command_velocity) < 1e-10:
            self.command_position = self.target_position
            self.command_velocity = 0.0
            self.publish()
            return
        direction = 1.0 if error >= 0.0 else -1.0
        stopping_distance = self.command_velocity**2 / (2.0 * max_acceleration)
        if self.command_velocity * direction < 0.0:
            acceleration = direction * max_acceleration
        elif abs(error) <= stopping_distance:
            acceleration = -math.copysign(max_acceleration, self.command_velocity)
        else:
            acceleration = direction * max_acceleration
        velocity = self.command_velocity + acceleration * dt
        velocity = max(-max_velocity, min(max_velocity, velocity))
        position = self.command_position + velocity * dt
        if direction * (self.target_position - position) <= 0.0:
            position = self.target_position
            velocity = 0.0
        self.command_position = position
        self.command_velocity = velocity
        self.publish()


def main() -> None:
    rclpy.init()
    node = Axis3IndexGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
