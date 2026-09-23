"""Watchdog-protected RPM guard for the Motor 6 conveyor."""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


PUBLIC_TOPIC = "/motor6_conveyor/commands_rpm"
RAW_TOPIC = "/motor6_raw_velocity_controller/commands"
HARD_MAX_RPM = 60.0
COMMISSIONING_MAX_RPM = 60.0
DEFAULT_ACCELERATION_RPM_S = 25.0
COMMAND_TIMEOUT_S = 0.5
UPDATE_PERIOD_S = 0.005


class Motor6ConveyorGuard(Node):
    def __init__(self) -> None:
        super().__init__("azd3a_motor6_conveyor_guard")
        self.declare_parameter("max_rpm", COMMISSIONING_MAX_RPM)
        self.declare_parameter("max_acceleration_rpm_s", DEFAULT_ACCELERATION_RPM_S)
        self.declare_parameter("command_timeout_s", COMMAND_TIMEOUT_S)
        self.max_rpm = float(self.get_parameter("max_rpm").value)
        self.max_acceleration_rpm_s = float(
            self.get_parameter("max_acceleration_rpm_s").value
        )
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        if not 0.0 < self.max_rpm <= HARD_MAX_RPM:
            raise ValueError("Motor 6 max_rpm must be within (0, 60]")
        if self.max_acceleration_rpm_s <= 0.0:
            raise ValueError("Motor 6 acceleration must be positive")
        if self.command_timeout_s <= UPDATE_PERIOD_S:
            raise ValueError("Motor 6 watchdog timeout is too short")

        self.target_rpm = 0.0
        self.output_rpm = 0.0
        self.acceleration_rpm_s = self.max_acceleration_rpm_s
        self.deceleration_rpm_s = self.max_acceleration_rpm_s
        self.last_command_time = None
        self.publisher = self.create_publisher(Float64MultiArray, RAW_TOPIC, 10)
        self.create_subscription(Float64MultiArray, PUBLIC_TOPIC, self.on_command, 10)
        self.create_timer(UPDATE_PERIOD_S, self.update)
        self.get_logger().info(
            f"Motor 6 conveyor guard ready: +/-{self.max_rpm:.1f} output rpm, "
            f"{self.max_acceleration_rpm_s:.1f} rpm/s ramp, "
            f"{self.command_timeout_s:.1f} s watchdog"
        )

    def on_command(self, message: Float64MultiArray) -> None:
        if len(message.data) not in (1, 3) or not all(math.isfinite(value) for value in message.data):
            self.get_logger().error("REJECTED Motor 6 command: expected [rpm] or [rpm, acceleration, deceleration]")
            return
        requested_rpm = float(message.data[0])
        if abs(requested_rpm) > self.max_rpm + 1e-9:
            self.get_logger().error(
                f"REJECTED Motor 6 command: {requested_rpm:.3f} rpm exceeds "
                f"+/-{self.max_rpm:.3f} rpm commissioning limit"
            )
            return
        if len(message.data) == 3:
            acceleration = float(message.data[1])
            deceleration = float(message.data[2])
            if not 0.0 < acceleration <= self.max_acceleration_rpm_s:
                self.get_logger().error("REJECTED Motor 6 acceleration: exceeds the configured guard limit")
                return
            if not 0.0 < deceleration <= self.max_acceleration_rpm_s:
                self.get_logger().error("REJECTED Motor 6 deceleration: exceeds the configured guard limit")
                return
            self.acceleration_rpm_s = acceleration
            self.deceleration_rpm_s = deceleration
        self.target_rpm = requested_rpm
        self.last_command_time = self.get_clock().now()

    def update(self) -> None:
        if self.last_command_time is None:
            self.target_rpm = 0.0
        else:
            age = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
            if age > self.command_timeout_s:
                self.target_rpm = 0.0
        ramp_rpm_s = (
            self.acceleration_rpm_s
            if abs(self.target_rpm) > abs(self.output_rpm)
            else self.deceleration_rpm_s
        )
        max_step = ramp_rpm_s * UPDATE_PERIOD_S
        difference = self.target_rpm - self.output_rpm
        self.output_rpm += max(-max_step, min(max_step, difference))
        message = Float64MultiArray()
        message.data = [self.output_rpm * math.pi / 30.0]
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = Motor6ConveyorGuard()
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
