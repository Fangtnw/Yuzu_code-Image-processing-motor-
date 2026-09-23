"""Runtime-zeroed, signed-angle guard for Motor 5 in the combined backend."""

import math

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from std_msgs.msg import Empty, Float64, Float64MultiArray


COMMAND_TOPIC = "/motor5_position_controller/commands_deg"
MANUAL_STEP_TOPIC = "/motor5_position_controller/manual_step_deg"
SET_ZERO_TOPIC = "/motor5_position_controller/set_zero"
RAW_TOPIC = "/motor5_raw_position_controller/commands"
JOINT_NAME = "motor4_joint"
INTERFACE_NAME = "motor5_position"
MAX_ANGLE_DEG = 180.0
MAX_VELOCITY_RPM = 20.0
MAX_ACCELERATION_RPM_S = 20.0
UPDATE_RATE_HZ = 200.0


class Motor5PositionGuard(Node):
    def __init__(self) -> None:
        super().__init__("azd3a_motor5_position_guard")
        self.measured_position = None
        self.origin = None
        self.command_position = None
        self.target_position = None
        self.command_velocity = 0.0
        self.manual_step_active = False
        self.raw_publisher = self.create_publisher(Float64MultiArray, RAW_TOPIC, 10)
        self.create_subscription(Float64, COMMAND_TOPIC, self.on_command, 10)
        self.create_subscription(Float64, MANUAL_STEP_TOPIC, self.on_manual_step, 10)
        self.create_subscription(Empty, SET_ZERO_TOPIC, self.on_set_zero, 10)
        self.create_subscription(
            DynamicJointState, "/dynamic_joint_states", self.on_dynamic_state, 10
        )
        self.create_timer(1.0 / UPDATE_RATE_HZ, self.update)
        self.get_logger().info(
            "Motor 5 guard ready: operator zero required, +/-180 degrees, "
            "20 rpm, 20 rpm/s"
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
        if self.origin is None and not self.manual_step_active:
            self.command_position = position
            self.target_position = position
            self.command_velocity = 0.0

    def on_set_zero(self, _message: Empty) -> None:
        if self.measured_position is None:
            self.get_logger().error("REJECTED Motor 5 zero: no valid feedback yet")
            return
        if abs(self.command_velocity) > 1e-9:
            self.get_logger().error("REJECTED Motor 5 zero: commanded motion is active")
            return
        self.origin = self.measured_position
        self.manual_step_active = False
        self.command_position = self.measured_position
        self.target_position = self.measured_position
        self.command_velocity = 0.0
        self.get_logger().info("Motor 5 runtime origin captured at current position")

    def on_command(self, message: Float64) -> None:
        offset_deg = message.data
        if not math.isfinite(offset_deg) or not -MAX_ANGLE_DEG <= offset_deg <= MAX_ANGLE_DEG:
            self.get_logger().error("REJECTED Motor 5 angle: permitted range is -180..180 degrees")
            return
        if self.origin is None or self.command_position is None:
            self.get_logger().error("REJECTED Motor 5 angle: set the runtime zero first")
            return
        self.target_position = self.origin + math.radians(offset_deg)
        self.get_logger().info(f"Accepted Motor 5 signed angle {offset_deg:.3f} deg")

    def on_manual_step(self, message: Float64) -> None:
        """Jog relative to the current position before the operator defines zero."""
        step_deg = message.data
        if not math.isfinite(step_deg) or not -MAX_ANGLE_DEG <= step_deg <= MAX_ANGLE_DEG:
            self.get_logger().error("REJECTED Motor 5 manual step: permitted range is -180..180 degrees")
            return
        if self.command_position is None or self.measured_position is None:
            self.get_logger().error("REJECTED Motor 5 manual step: no valid feedback yet")
            return
        self.target_position = self.command_position + math.radians(step_deg)
        self.manual_step_active = True
        self.get_logger().info(f"Accepted Motor 5 manual step {step_deg:+.3f} deg")

    def publish(self) -> None:
        message = Float64MultiArray()
        message.data = [self.command_position]
        self.raw_publisher.publish(message)

    def update(self) -> None:
        if self.command_position is None or self.target_position is None:
            return
        dt = 1.0 / UPDATE_RATE_HZ
        error = self.target_position - self.command_position
        max_velocity = MAX_VELOCITY_RPM * math.pi / 30.0
        max_acceleration = MAX_ACCELERATION_RPM_S * math.pi / 30.0
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
    node = Motor5PositionGuard()
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
