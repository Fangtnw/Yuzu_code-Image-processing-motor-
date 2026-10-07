"""Watchdog-protected cyclic synchronous position indexing for Motor 6."""

import json
import math
import time
import uuid
from collections import OrderedDict

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from motor_controller.conveyor_index import (
    ConveyorIndex, COUNTS_PER_RAD, FEEDBACK_TIMEOUT_S, MM_PER_COUNT, NOMINAL_LOOP_MM, OPERATING_MAX_RPM,
)


PUBLIC_TOPIC = "/motor6_conveyor/commands_rpm"
RAW_TOPIC = "/motor6_raw_position_controller/commands"
INDEX_TOPIC = "/motor6_conveyor/index_request"
HEARTBEAT_TOPIC = "/motor6_conveyor/index_heartbeat"
STATUS_TOPIC = "/motor6_conveyor/index_status"
HARD_MAX_RPM = 60.0
COMMISSIONING_MAX_RPM = OPERATING_MAX_RPM
DEFAULT_ACCELERATION_RPM_S = 96.0
COMMAND_TIMEOUT_S = 0.5
UPDATE_PERIOD_S = 0.005


class Motor6ConveyorGuard(Node):
    def __init__(self) -> None:
        super().__init__("azd3a_motor6_conveyor_guard")
        defaults = {
            "max_rpm": COMMISSIONING_MAX_RPM,
            "max_acceleration_rpm_s": DEFAULT_ACCELERATION_RPM_S,
            "command_timeout_s": COMMAND_TIMEOUT_S,
            "index_loop_counts": NOMINAL_LOOP_MM / MM_PER_COUNT,
            "index_tolerance_mm": 0.02,
            "feedback_joint": "motor4_joint",
            "position_interface": "motor6_position",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        parameter = lambda name: self.get_parameter(name).value
        self.motion = ConveyorIndex(
            max_rpm=float(parameter("max_rpm")),
            max_ramp=float(parameter("max_acceleration_rpm_s")),
            command_timeout=float(parameter("command_timeout_s")),
            loop_counts=float(parameter("index_loop_counts")),
            tolerance_mm=float(parameter("index_tolerance_mm")),
        )
        self.feedback_joint = str(parameter("feedback_joint"))
        self.position_interface = str(parameter("position_interface"))
        self.session = uuid.uuid4().hex
        self.lease = None
        self.requests = OrderedDict()
        self.last_request = {"request_id": None, "request_ok": None, "request_message": ""}
        self.last_feedback_stamp = None
        self.startup_position_counts = None
        self.startup_stable_since = None
        self.last_stop_stamp = 0
        self.publisher = self.create_publisher(Float64MultiArray, RAW_TOPIC, 1)
        self.status_publisher = self.create_publisher(String, STATUS_TOPIC, 1)
        self.create_subscription(Float64MultiArray, PUBLIC_TOPIC, self.on_command, 1)
        self.create_subscription(String, INDEX_TOPIC, self.on_index_request, 1)
        self.create_subscription(String, HEARTBEAT_TOPIC, self.on_heartbeat, 1)
        self.create_subscription(DynamicJointState, "/dynamic_joint_states", self.on_feedback, 1)
        self.create_timer(UPDATE_PERIOD_S, self.update)
        self.create_timer(0.1, self.publish_status)
        self.get_logger().info(
            f"Motor 6 guard: {self.motion.max_rpm:g} rpm cap, 12 fixtures, "
            f"{self.motion.loop_counts * MM_PER_COUNT / 12:.9f} nominal mm/index; "
            "align a fixture and capture reference before indexing"
        )

    def on_feedback(self, message: DynamicJointState) -> None:
        try:
            joint = message.joint_names.index(self.feedback_joint)
            interfaces = message.interface_values[joint]
            values = dict(zip(interfaces.interface_names, interfaces.values))
            position = values[self.position_interface]
            velocity = values["velocity"]
            status = values["motor6_status"]
            if not all(math.isfinite(v) for v in (position, velocity, status)):
                raise ValueError("Non-finite feedback")
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if stamp <= 0 or not -0.02 <= age <= FEEDBACK_TIMEOUT_S:
                raise ValueError("Stale/missing feedback timestamp")
            if self.last_feedback_stamp is not None and stamp <= self.last_feedback_stamp:
                return
        except (ValueError, KeyError, IndexError):
            # Other joints/status messages must not refresh Motor 6 position.
            return
        self.last_feedback_stamp = stamp
        now = time.monotonic() - max(0.0, age)
        counts = round(position * COUNTS_PER_RAD)
        enabled = (int(status) & 0x006F) == 0x0027
        if self.motion.position_counts is None:
            # Joint-state startup may expose zero-initialized values before
            # the first real EtherCAT sample. Do not latch those as a target.
            if not enabled or abs(velocity * 30.0 / math.pi) > 0.01:
                self.startup_position_counts = self.startup_stable_since = None
                return
            if self.startup_position_counts is None or abs(counts - self.startup_position_counts) > 2:
                self.startup_position_counts = counts
                self.startup_stable_since = now
                return
            if now - self.startup_stable_since < 0.3:
                return
        self.motion.feedback(
            counts, velocity * 30.0 / math.pi, enabled, now,
        )

    def on_command(self, message: Float64MultiArray) -> None:
        if len(message.data) not in (1, 3) or not all(math.isfinite(v) for v in message.data):
            self.get_logger().error("REJECTED Motor 6: expected [rpm] or [rpm, acceleration, deceleration]")
            return
        try:
            if message.data[0] != 0.0:
                raise ValueError("Motor 6 is in CSP position mode; use an indexed fixture request")
            acceleration, deceleration = (
                message.data[1:] if len(message.data) == 3
                else (self.motion.acceleration, self.motion.deceleration)
            )
            self.motion.manual(0.0, acceleration, deceleration, time.monotonic())
            self.last_stop_stamp = self.get_clock().now().nanoseconds
            self.lease = None
        except ValueError as error:
            self.get_logger().error(f"REJECTED Motor 6: {error}")
        self.publish_status()

    def on_index_request(self, message: String) -> None:
        request_id = None
        try:
            request = json.loads(message.data)
            if not isinstance(request, dict):
                raise ValueError("Index request must be a JSON object")
            request_id = request.get("id")
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 64:
                raise ValueError("Missing/invalid request ID")
            if request.get("session") != self.session:
                raise ValueError("Backend restarted; refresh status and re-reference")
            if request_id in self.requests:
                self.last_request = self.requests[request_id]
                self.publish_status()
                return
            sent = request.get("sent_ns")
            if not isinstance(sent, int) or isinstance(sent, bool):
                raise ValueError("Missing request timestamp")
            age = (self.get_clock().now().nanoseconds - sent) / 1e9
            if not -0.02 <= age <= 0.5 or sent <= self.last_stop_stamp:
                raise ValueError("Request expired or predates the last stop; issue a new request")
            operation = request.get("op")
            now = time.monotonic()
            was_indexing = self.motion.state == "indexing"
            if operation == "reference":
                self.motion.capture_reference(now)
            else:
                if request.get("reference_id") != self.motion.reference_id:
                    raise ValueError("Fixture reference changed; refresh status")
                if operation == "index":
                    index = request["index"]
                    if isinstance(index, bool) or not isinstance(index, int):
                        raise ValueError("Fixture index must be an integer")
                    self.motion.request_index(index, float(request["rpm"]),
                                              float(request["acceleration"]),
                                              float(request["deceleration"]), now)
                elif operation == "jog":
                    self.motion.jog(float(request["distance_mm"]), float(request["rpm"]),
                                    float(request["acceleration"]), float(request["deceleration"]), now)
                elif operation == "resume":
                    self.motion.resume(now)
                else:
                    raise ValueError("Unknown conveyor index operation")
            if self.motion.state in ("indexing", "jogging") and not was_indexing:
                self.lease = request_id
            ok, detail = True, self.motion.message
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            ok, detail = False, str(error)
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 64:
                request_id = None
            self.get_logger().error(f"REJECTED conveyor index request: {detail}")
        self.last_request = {"request_id": request_id, "request_ok": ok, "request_message": detail}
        if isinstance(request_id, str) and 1 <= len(request_id) <= 64:
            self.requests[request_id] = self.last_request
            if len(self.requests) > 128:
                self.requests.popitem(last=False)
        self.publish_status()

    def on_heartbeat(self, message: String) -> None:
        if self.lease is not None and message.data == self.lease:
            self.motion.heartbeat(time.monotonic())

    def update(self) -> None:
        rpm = self.motion.update(time.monotonic())
        if self.motion.state not in ("indexing", "jogging"):
            self.lease = None
        message = Float64MultiArray()
        # Forward the absolute position trajectory in output-shaft radians.
        # Motor 6 is configured in CSP (0x6060=8); the drive, rather than a
        # late velocity stop, closes the final position error.
        if self.motion.command_counts is None:
            return  # Driver holds live feedback until a synchronized target exists.
        command_position = ((round(self.motion.command_counts) + 2**31) % 2**32 - 2**31) / COUNTS_PER_RAD
        message.data = [command_position]
        self.publisher.publish(message)

    def publish_status(self) -> None:
        status = self.motion.status(time.monotonic())
        status.update(self.last_request)
        status.update(session=self.session, position_counts=self.motion.position_counts,
                      origin_counts=self.motion.origin_counts, target_counts=self.motion.target_counts)
        message = String()
        message.data = json.dumps(status, allow_nan=False)
        self.status_publisher.publish(message)


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
