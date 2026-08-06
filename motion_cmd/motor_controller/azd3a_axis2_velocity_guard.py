"""Guard and ramp Axis 2 commissioning velocity commands."""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


PUBLIC_TOPIC = "/axis2_velocity_controller/commands_rpm"
RAW_TOPIC = "/axis2_raw_velocity_controller/commands"
HARD_MAX_RPM = 416.0
COMMISSIONING_MAX_RPM = 25.0
DEFAULT_ACCELERATION_RPM_S = 10.0
COMMAND_TIMEOUT_S = 0.5
UPDATE_PERIOD_S = 0.005


class Axis2VelocityGuard(Node):
    def __init__(self) -> None:
        super().__init__("azd3a_axis2_velocity_guard")
        self.declare_parameter("max_rpm", COMMISSIONING_MAX_RPM)
        self.declare_parameter("max_acceleration_rpm_s", DEFAULT_ACCELERATION_RPM_S)
        self.declare_parameter("command_timeout_s", COMMAND_TIMEOUT_S)

        self.max_rpm = float(self.get_parameter("max_rpm").value)
        self.max_acceleration_rpm_s = float(
            self.get_parameter("max_acceleration_rpm_s").value
        )
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        if not 0.0 < self.max_rpm <= HARD_MAX_RPM:
            raise ValueError(f"max_rpm must be within (0, {HARD_MAX_RPM}]")
        if self.max_acceleration_rpm_s <= 0.0:
            raise ValueError("max_acceleration_rpm_s must be positive")
        if self.command_timeout_s <= UPDATE_PERIOD_S:
            raise ValueError("command_timeout_s is too short")

        self.target_rpm = 0.0
        self.output_rpm = 0.0
        self.last_command_time = None
        self.publisher = self.create_publisher(Float64MultiArray, RAW_TOPIC, 10)
        self.subscription = self.create_subscription(
            Float64MultiArray, PUBLIC_TOPIC, self.command_callback, 10
        )
        self.timer = self.create_timer(UPDATE_PERIOD_S, self.update)
        self.get_logger().info(
            f"Axis 2 guard ready: +/-{self.max_rpm:.1f} rpm, "
            f"{self.max_acceleration_rpm_s:.1f} rpm/s ramp, "
            f"{self.command_timeout_s:.1f} s watchdog"
        )

    def command_callback(self, message: Float64MultiArray) -> None:
        if len(message.data) != 1 or not math.isfinite(message.data[0]):
            self.get_logger().error("REJECTED: expected one finite rpm value")
            return
        requested_rpm = float(message.data[0])
        if abs(requested_rpm) > self.max_rpm + 1e-9:
            self.get_logger().error(
                f"REJECTED: {requested_rpm:.3f} rpm exceeds "
                f"+/-{self.max_rpm:.3f} rpm commissioning limit"
            )
            return
        self.target_rpm = requested_rpm
        self.last_command_time = self.get_clock().now()

    def update(self) -> None:
        if self.last_command_time is None:
            self.target_rpm = 0.0
        else:
            age = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
            if age > self.command_timeout_s:
                self.target_rpm = 0.0

        max_step = self.max_acceleration_rpm_s * UPDATE_PERIOD_S
        difference = self.target_rpm - self.output_rpm
        self.output_rpm += max(-max_step, min(max_step, difference))
        message = Float64MultiArray()
        message.data = [self.output_rpm * math.pi / 30.0]
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = Axis2VelocityGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
