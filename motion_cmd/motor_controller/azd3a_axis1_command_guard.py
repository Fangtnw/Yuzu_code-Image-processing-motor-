"""Bounded, velocity/acceleration-limited command proxy for AZD3A Axis 1."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# Manufacturer envelope for DR28T1A03-AZAKR. These are hard caps, not defaults.
HARD_MIN_POSITION_M = 0.0
HARD_MAX_POSITION_M = 0.030
HARD_MAX_VELOCITY_M_S = 0.040
HARD_MAX_ACCELERATION_M_S2 = 0.2
MIN_TRAVEL_INCREMENT_M = 0.000001

# Conservative commissioning defaults. Launch parameters may relax them only
# within the manufacturer envelope above.
DEFAULT_MAX_POSITION_M = 0.005
DEFAULT_MAX_VELOCITY_M_S = 0.0005
DEFAULT_MAX_ACCELERATION_M_S2 = 0.001
UPDATE_RATE_HZ = 200.0
PUBLIC_TOPIC = "/axis1_position_controller/commands"
RAW_TOPIC = "/axis1_raw_position_controller/commands"


class Axis1CommandGuard(Node):
    def __init__(self) -> None:
        super().__init__("azd3a_axis1_command_guard")
        self.measured_position = None
        self.command_position = None
        self.target_position = None
        self.command_velocity = 0.0
        self.operator_command_received = False

        self.min_position = self.declare_parameter(
            "min_position_m", HARD_MIN_POSITION_M
        ).value
        self.max_position = self.declare_parameter(
            "max_position_m", DEFAULT_MAX_POSITION_M
        ).value
        self.max_velocity = self.declare_parameter(
            "max_velocity_m_s", DEFAULT_MAX_VELOCITY_M_S
        ).value
        self.max_acceleration = self.declare_parameter(
            "max_acceleration_m_s2", DEFAULT_MAX_ACCELERATION_M_S2
        ).value
        self.validate_limits()

        self.raw_publisher = self.create_publisher(Float64MultiArray, RAW_TOPIC, 10)
        self.create_subscription(Float64MultiArray, PUBLIC_TOPIC, self.on_command, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_timer(1.0 / UPDATE_RATE_HZ, self.update_command)

        self.get_logger().info(
            f"Axis 1 guard active: position {self.min_position * 1000:.3f}.."
            f"{self.max_position * 1000:.3f} mm, velocity "
            f"{self.max_velocity * 1000:.3f} mm/s, acceleration "
            f"{self.max_acceleration:.3f} m/s^2"
        )

    def validate_limits(self) -> None:
        if not (
            HARD_MIN_POSITION_M <= self.min_position < self.max_position
            <= HARD_MAX_POSITION_M
        ):
            raise ValueError("Configured position range exceeds the 0..30 mm stroke")
        if not 0.0 < self.max_velocity <= HARD_MAX_VELOCITY_M_S:
            raise ValueError("Configured velocity exceeds the 40 mm/s specification")
        if not 0.0 < self.max_acceleration <= HARD_MAX_ACCELERATION_M_S2:
            raise ValueError("Configured acceleration exceeds the 0.2 m/s^2 specification")

    def on_joint_state(self, message: JointState) -> None:
        try:
            index = message.name.index("axis1_joint")
            position = message.position[index]
        except (ValueError, IndexError):
            return
        if not math.isfinite(position):
            return
        self.measured_position = position
        if not self.operator_command_received:
            # Before the first operator target, continuously follow feedback.
            # EtherCAT may publish a temporary zero before the slave reaches
            # OP; latching only the first sample would preserve that false
            # zero and prevent safe synchronization at a retained position.
            self.command_position = position
            self.target_position = position
            self.command_velocity = 0.0

    def on_command(self, message: Float64MultiArray) -> None:
        if len(message.data) != 1 or not math.isfinite(message.data[0]):
            self.get_logger().error("REJECTED Axis 1 command: expected one finite value")
            return
        target = message.data[0]
        if not self.min_position <= target <= self.max_position:
            self.get_logger().error(
                f"REJECTED Axis 1 target {target:.6f} m: permitted range is "
                f"{self.min_position:.3f}..{self.max_position:.3f} m"
            )
            return
        increments = round(target / MIN_TRAVEL_INCREMENT_M)
        if abs(target - increments * MIN_TRAVEL_INCREMENT_M) > 1e-10:
            self.get_logger().error(
                "REJECTED Axis 1 target: value must use 0.001 mm increments"
            )
            return
        if self.command_position is None:
            self.get_logger().error("REJECTED Axis 1 command: no measured feedback yet")
            return
        self.operator_command_received = True
        self.target_position = target
        self.get_logger().info(f"Accepted Axis 1 target {target * 1000:.3f} mm")

    def publish_raw(self) -> None:
        message = Float64MultiArray()
        message.data = [self.command_position]
        self.raw_publisher.publish(message)

    def update_command(self) -> None:
        if self.command_position is None or self.target_position is None:
            return

        dt = 1.0 / UPDATE_RATE_HZ
        error = self.target_position - self.command_position
        if abs(error) < 1e-10 and abs(self.command_velocity) < 1e-10:
            self.command_position = self.target_position
            self.command_velocity = 0.0
            self.publish_raw()
            return

        direction = 1.0 if error >= 0.0 else -1.0
        stopping_distance = (
            self.command_velocity * self.command_velocity
            / (2.0 * self.max_acceleration)
        )

        if self.command_velocity * direction < 0.0:
            acceleration = direction * self.max_acceleration
        elif abs(error) <= stopping_distance:
            acceleration = -math.copysign(
                self.max_acceleration, self.command_velocity
            )
        else:
            acceleration = direction * self.max_acceleration

        next_velocity = self.command_velocity + acceleration * dt
        next_velocity = max(
            -self.max_velocity, min(self.max_velocity, next_velocity)
        )
        next_position = self.command_position + next_velocity * dt

        if direction * (self.target_position - next_position) <= 0.0:
            next_position = self.target_position
            next_velocity = 0.0

        self.command_position = next_position
        self.command_velocity = next_velocity
        self.publish_raw()


def main() -> None:
    rclpy.init()
    node = Axis1CommandGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
