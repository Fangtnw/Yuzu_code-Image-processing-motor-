"""Absolute-position guard for machine Motor 4 at its required speed."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


PUBLIC_TOPIC = "/motor4_position_controller/commands_mm"
RAW_TOPIC = "/motor4_raw_position_controller/commands"
JOINT_NAME = "motor4_joint"
MIN_POSITION_MM = 0.0
MAX_POSITION_MM = 15.0
MIN_INCREMENT_MM = 0.001
HARD_MAX_POSITION_MM = 30.0
HARD_MAX_VELOCITY_M_S = 0.040
HARD_MAX_ACCELERATION_M_S2 = 0.2
UPDATE_RATE_HZ = 200.0
REQUIRED_VELOCITY_M_S = 0.008
DEFAULT_ACCELERATION_M_S2 = 0.050


class Motor4PositionGuard(Node):
    def __init__(self) -> None:
        super().__init__("azd3a_motor4_position_guard")
        self.measured_position = None
        self.command_position = None
        self.target_position = None
        self.command_velocity = 0.0
        self.active_max_velocity = REQUIRED_VELOCITY_M_S
        self.active_max_acceleration = DEFAULT_ACCELERATION_M_S2
        self.operator_command_received = False

        self.raw_publisher = self.create_publisher(Float64MultiArray, RAW_TOPIC, 10)
        self.create_subscription(Float64, PUBLIC_TOPIC, self.on_command, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_timer(1.0 / UPDATE_RATE_HZ, self.update_command)
        self.get_logger().info(
            "Motor 4 guard ready: absolute 0..15 mm, 0.001 mm increments; "
            "8 mm/s velocity, 50 mm/s^2 acceleration/deceleration"
        )

    def on_joint_state(self, message: JointState) -> None:
        try:
            position = message.position[message.name.index(JOINT_NAME)]
        except (ValueError, IndexError):
            return
        if not math.isfinite(position):
            return
        self.measured_position = position
        if not self.operator_command_received:
            # Follow valid feedback until the first command so enabling CSP
            # cannot produce a retained-position startup jump.
            self.command_position = position
            self.target_position = position
            self.command_velocity = 0.0

    def on_command(self, message: Float64) -> None:
        target_mm = message.data
        if not math.isfinite(target_mm):
            self.get_logger().error("REJECTED Motor 4 target: expected a finite value")
            return
        if not MIN_POSITION_MM <= target_mm <= MAX_POSITION_MM:
            self.get_logger().error("REJECTED Motor 4 target: permitted range is 0..15 mm")
            return
        increments = round(target_mm / MIN_INCREMENT_MM)
        if abs(target_mm - increments * MIN_INCREMENT_MM) > 1e-9:
            self.get_logger().error("REJECTED Motor 4 target: use 0.001 mm increments")
            return
        if self.command_position is None or self.measured_position is None:
            self.get_logger().error("REJECTED Motor 4 target: no valid feedback yet")
            return

        target_position = target_mm / 1000.0
        distance_mm = abs(target_position - self.measured_position) * 1000.0
        velocity = REQUIRED_VELOCITY_M_S
        acceleration = DEFAULT_ACCELERATION_M_S2
        if velocity > HARD_MAX_VELOCITY_M_S or acceleration > HARD_MAX_ACCELERATION_M_S2:
            self.get_logger().error("REJECTED Motor 4 target: internal profile exceeds hardware limits")
            return

        self.operator_command_received = True
        self.target_position = target_position
        self.active_max_velocity = velocity
        self.active_max_acceleration = acceleration
        self.get_logger().info(
            f"Accepted Motor 4 target {target_mm:.3f} mm over {distance_mm:.3f} mm: "
            f"{velocity * 1000.0:.1f} mm/s, {acceleration:.3f} m/s^2"
        )

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
        stopping_distance = self.command_velocity**2 / (2.0 * self.active_max_acceleration)
        if self.command_velocity * direction < 0.0:
            acceleration = direction * self.active_max_acceleration
        elif abs(error) <= stopping_distance:
            acceleration = -math.copysign(self.active_max_acceleration, self.command_velocity)
        else:
            acceleration = direction * self.active_max_acceleration

        velocity = self.command_velocity + acceleration * dt
        velocity = max(-self.active_max_velocity, min(self.active_max_velocity, velocity))
        position = self.command_position + velocity * dt
        if direction * (self.target_position - position) <= 0.0:
            position = self.target_position
            velocity = 0.0
        self.command_position = position
        self.command_velocity = velocity
        self.publish_raw()


def main() -> None:
    rclpy.init()
    node = Motor4PositionGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
