"""One slow, bounded Axis 1 round trip for recording a progress video."""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


FORWARD_TARGET_M = 0.005
RETURN_TARGET_M = 0.001
SPEED_M_PER_S = 0.0005
PUBLISH_RATE_HZ = 20.0
MAX_START_POSITION_M = 0.006
MIN_START_POSITION_M = -0.0001


class Axis1VideoDemo(Node):
    def __init__(self) -> None:
        super().__init__("azd3a_axis1_video_demo")
        self.position = None
        self.publisher = self.create_publisher(
            Float64MultiArray,
            "/axis1_position_controller/commands",
            10,
        )
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)

    def on_joint_state(self, message: JointState) -> None:
        try:
            index = message.name.index("axis1_joint")
            position = message.position[index]
        except (ValueError, IndexError):
            return
        if math.isfinite(position):
            self.position = position

    def publish_position(self, position: float) -> None:
        message = Float64MultiArray()
        message.data = [position]
        self.publisher.publish(message)

    def wait_for_feedback(self, timeout: float = 5.0) -> float:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.position is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.position is None:
            raise RuntimeError("No finite axis1_joint feedback received within 5 seconds")
        return self.position

    def ramp_to(self, target: float) -> None:
        start = self.position
        step = SPEED_M_PER_S / PUBLISH_RATE_HZ
        direction = 1.0 if target >= start else -1.0
        command = start
        self.get_logger().info(
            f"Ramping from {start * 1000:.3f} mm to {target * 1000:.3f} mm"
        )
        while rclpy.ok() and direction * (target - command) > 0:
            command += direction * min(step, abs(target - command))
            self.publish_position(command)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / PUBLISH_RATE_HZ)
        self.publish_position(target)
        time.sleep(0.5)


def main() -> None:
    rclpy.init()
    node = Axis1VideoDemo()
    try:
        start = node.wait_for_feedback()
        if not MIN_START_POSITION_M <= start <= MAX_START_POSITION_M:
            raise RuntimeError(
                f"Start position {start:.7f} m is outside the permitted "
                f"{MIN_START_POSITION_M}..{MAX_START_POSITION_M} m demo region"
            )
        print(
            "Planned motion: slowly move Axis 1 to 5 mm, then return to 1 mm.\n"
            "The mechanism must be clear and the physical power cutoff accessible."
        )
        if input('Type exactly "VIDEO MOVE" to continue: ') != "VIDEO MOVE":
            raise RuntimeError("Cancelled; no demo motion was commanded")
        node.ramp_to(FORWARD_TARGET_M)
        node.ramp_to(RETURN_TARGET_M)
        node.get_logger().info("Video movement complete; final target is 1.000 mm")
    except (RuntimeError, KeyboardInterrupt) as error:
        node.get_logger().error(str(error))
        sys.exit_code = 2
    else:
        sys.exit_code = 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(sys.exit_code)


if __name__ == "__main__":
    main()
