"""Absolute Motor 3 guard for the combined three-axis slave-0 backend."""

import math

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


PUBLIC_TOPIC = "/motor3_position_controller/commands_mm"
RAW_TOPIC = "/motor3_raw_position_controller/commands"
JOINT_NAME = "motor1_motor2"
INTERFACE_NAME = "motor3_position"
MIN_POSITION_MM = 0.0
MAX_POSITION_MM = 20.0
MIN_INCREMENT_MM = 0.0001
MAX_VELOCITY_M_S = 0.008
MAX_ACCELERATION_M_S2 = 0.050
UPDATE_RATE_HZ = 200.0


class Motor3PositionGuard(Node):
    def __init__(self) -> None:
        super().__init__("azd3a_motor3_position_guard")
        self.measured_position = None
        self.command_position = None
        self.target_position = None
        self.command_velocity = 0.0
        self.operator_command_received = False
        self.raw_publisher = self.create_publisher(Float64MultiArray, RAW_TOPIC, 10)
        self.create_subscription(Float64, PUBLIC_TOPIC, self.on_command, 10)
        self.create_subscription(
            DynamicJointState, "/dynamic_joint_states", self.on_dynamic_state, 10
        )
        self.create_timer(1.0 / UPDATE_RATE_HZ, self.update_command)
        self.get_logger().info(
            "Motor 3 combined guard ready: 0..20 mm, 8 mm/s, 50 mm/s^2"
        )

    def on_dynamic_state(self, message: DynamicJointState) -> None:
        try:
            joint_index = message.joint_names.index(JOINT_NAME)
            interfaces = message.interface_values[joint_index]
            interface_index = interfaces.interface_names.index(INTERFACE_NAME)
            position = interfaces.values[interface_index]
        except (ValueError, IndexError):
            return
        if not math.isfinite(position):
            return
        self.measured_position = position
        if not self.operator_command_received:
            self.command_position = position
            self.target_position = position
            self.command_velocity = 0.0

    def on_command(self, message: Float64) -> None:
        target_mm = message.data
        if not math.isfinite(target_mm) or not MIN_POSITION_MM <= target_mm <= MAX_POSITION_MM:
            self.get_logger().error("REJECTED Motor 3 target: permitted range is 0..20 mm")
            return
        increments = round(target_mm / MIN_INCREMENT_MM)
        if abs(target_mm - increments * MIN_INCREMENT_MM) > 1e-9:
            self.get_logger().error("REJECTED Motor 3 target: use 0.0001 mm increments")
            return
        if self.command_position is None or self.measured_position is None:
            self.get_logger().error("REJECTED Motor 3 target: no valid feedback yet")
            return
        self.operator_command_received = True
        self.target_position = target_mm / 1000.0
        self.get_logger().info(f"Accepted Motor 3 target {target_mm:.3f} mm")

    def update_command(self) -> None:
        if self.command_position is None or self.target_position is None:
            return
        dt = 1.0 / UPDATE_RATE_HZ
        error = self.target_position - self.command_position
        if abs(error) < 1e-12 and abs(self.command_velocity) < 1e-12:
            self.command_position = self.target_position
            self.command_velocity = 0.0
        else:
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
        message = Float64MultiArray()
        message.data = [self.command_position]
        self.raw_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = Motor3PositionGuard()
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
