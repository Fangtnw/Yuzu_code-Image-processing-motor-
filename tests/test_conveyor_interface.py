"""Execute real ROS callback/GUI methods with transport and widget doubles."""

import ast
from collections import OrderedDict
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1] / "motion_cmd"
sys.path.insert(0, str(ROOT))
from motor_controller.conveyor_index import ConveyorIndex, COUNTS_PER_RAD  # noqa: E402


def load_without_ros(filename):
    """Only substitute middleware imports, leaving all executable methods intact."""
    path = ROOT / "motor_controller" / filename
    tree = ast.parse(path.read_text())
    ros_modules = ("rclpy", "control_msgs", "sensor_msgs", "std_msgs")
    tree.body = [n for n in tree.body if not (
        isinstance(n, ast.ImportFrom) and n.module.split('.')[0] in ros_modules
        or isinstance(n, ast.Import) and any(a.name.split('.')[0] in ros_modules for a in n.names)
    )]
    module = ModuleType("offline_" + path.stem)
    module.__dict__.update(Node=object, DynamicJointState=NS, JointState=NS,
                           Float64MultiArray=NS, Float64=NS, Empty=NS, String=NS)
    exec(compile(tree, str(path), "exec"), module.__dict__)
    return module


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return 1


class Value:
    def __init__(self, value=""):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class ConveyorInterfaceTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.5
        self.module = load_without_ros("azd3a_motor6_conveyor_guard.py")
        self.module.time = NS(monotonic=lambda: self.now)
        self.guard = self.module.Motor6ConveyorGuard.__new__(self.module.Motor6ConveyorGuard)
        g = self.guard
        g.motion = ConveyorIndex()
        for tick in range(100):
            g.motion.feedback(1_000_000, 0.0, True, tick * .005)
            g.motion.update(tick * .005)
        g.session = "test-session"
        g.lease = None
        g.requests = OrderedDict()
        g.last_request = {"request_id": None, "request_ok": None, "request_message": ""}
        g.last_feedback_stamp = None
        g.last_stop_stamp = 0
        g.feedback_joint = "motor4_joint"
        g.position_interface = "motor6_position"
        g.publisher = Publisher()
        g.status_publisher = Publisher()
        self.errors = []
        g.get_logger = lambda: NS(error=self.errors.append)
        g.get_clock = lambda: NS(now=lambda: NS(nanoseconds=self.stamp()))

    def stamp(self):
        return 2_000_000_000 + round(self.now * 1e9)

    def request(self, **overrides):
        data = dict(id="request-1", session=self.guard.session,
                    reference_id=self.guard.motion.reference_id, op="index", index=1,
                    rpm=48.0, acceleration=96.0, deceleration=96.0, sent_ns=self.stamp())
        data.update(overrides)
        return NS(data=json.dumps(data))

    def feedback_message(self, names=None, values=None, stamp=None):
        stamp = self.stamp() if stamp is None else stamp
        return NS(header=NS(stamp=NS(sec=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000)),
                  joint_names=[self.guard.feedback_joint], interface_values=[NS(
                      interface_names=names or [self.guard.position_interface, "velocity", "motor6_status"],
                      values=values or [1_000_000 / COUNTS_PER_RAD, 0.0, 0x27])])

    def test_request_ack_retry_and_heartbeat_ownership(self):
        request = self.request()
        self.guard.on_index_request(request)
        self.assertTrue(self.guard.last_request["request_ok"])
        self.assertEqual(self.guard.motion.state, "indexing")
        target = self.guard.motion.target_counts
        self.guard.on_index_request(request)
        self.assertEqual(self.guard.motion.target_counts, target)
        self.now += .01
        self.guard.on_heartbeat(NS(data="wrong-owner"))
        self.assertNotEqual(self.guard.motion.last_command, self.now)
        self.guard.on_heartbeat(NS(data="request-1"))
        self.assertEqual(self.guard.motion.last_command, self.now)
        self.guard.on_index_request(self.request(id="duplicate-index"))
        self.assertEqual(self.guard.lease, "request-1")
        self.guard.on_command(NS(data=[0.0]))
        self.guard.on_index_request(request)
        self.assertEqual(self.guard.motion.state, "interrupted")
        self.assertIsNone(self.guard.lease)

    def test_request_queued_before_stop_cannot_start_after_stop(self):
        request = self.request()
        self.now += .001
        self.guard.on_command(NS(data=[0.0]))
        self.guard.on_index_request(request)
        self.assertFalse(self.guard.last_request["request_ok"])
        self.assertEqual(self.guard.motion.state, "unreferenced")

    def test_bad_session_reference_payload_and_stale_request_cannot_move(self):
        for index, overrides in enumerate((dict(session="previous-boot"), dict(reference_id=999),
                                           dict(sent_ns=1), dict(rpm=float("nan")),
                                           dict(index=True), dict(index=2))):
            self.guard.on_index_request(self.request(id=f"bad-{index}", **overrides))
            self.assertFalse(self.guard.last_request["request_ok"])
            self.assertEqual(self.guard.motion.state, "unreferenced")
        for data in ("{", "[]", "null", '{"id": NaN}'):
            self.guard.on_index_request(NS(data=data))
            self.assertEqual(self.guard.motion.state, "unreferenced")

    def test_only_complete_fresh_position_velocity_and_status_refresh_readiness(self):
        self.guard.on_index_request(self.request())
        initial_stamp = self.guard.motion.feedback_time
        for tick in range(1, 31):
            self.now = .5 + tick * .005
            self.guard.on_feedback(self.feedback_message(names=["motor6_status"], values=[0x27]))
            self.guard.update()
        self.assertEqual(self.guard.motion.feedback_time, initial_stamp)
        self.assertEqual(self.guard.motion.state, "interrupted")
        self.guard.on_feedback(self.feedback_message(stamp=self.stamp() - 200_000_000))
        self.assertEqual(self.guard.motion.feedback_time, initial_stamp)

    def test_combined_and_standalone_dynamic_feedback_mapping(self):
        for joint, position in (("motor4_joint", "motor6_position"), ("motor6_joint", "position")):
            self.guard.motion = ConveyorIndex()
            self.guard.last_feedback_stamp = None
            self.guard.feedback_joint, self.guard.position_interface = joint, position
            self.guard.on_feedback(self.feedback_message())
            self.assertEqual(self.guard.motion.position_counts, 1_000_000)
            self.assertTrue(self.guard.motion.fresh(self.now))
            previous = self.guard.motion.feedback_time
            self.now += .01
            self.guard.on_feedback(self.feedback_message(values=[float("nan"), 0.0, 0x27]))
            self.assertEqual(self.guard.motion.feedback_time, previous)
            self.now += .01
            self.guard.on_feedback(self.feedback_message(values=[1_000_000 / COUNTS_PER_RAD, 0.0, 0x38]))
            self.assertFalse(self.guard.motion.fresh(self.now))


class ConveyorGuiTests(unittest.TestCase):
    def setUp(self):
        self.module = load_without_ros("yuzu_peeler_gui.py")
        self.now = 0.5
        self.module.time = NS(monotonic=lambda: self.now)
        self.errors = []
        self.module.messagebox = NS(showerror=lambda title, text: self.errors.append(text),
                                    askyesno=lambda *args: True)
        self.gui = self.module.YuzuOperatorGui.__new__(self.module.YuzuOperatorGui)
        gui = self.gui
        self.sent, self.heartbeats, self.stops = [], [], []
        gui.node = NS(motor6_index_status={"session": "test", "reference_id": 0, "index": 0,
                                          "referenced": False, "state": "unreferenced",
                                          "stopped": True, "fresh": True, "pitch_mm": 140.825},
                      motor6_status_fresh=lambda: True, motor6_index_publisher=Publisher(),
                      publish_motor6_index=self.sent.append,
                      publish_motor6_heartbeat=self.heartbeats.append,
                      publish_motor6=self.stops.append,
                      get_clock=lambda: NS(now=lambda: NS(nanoseconds=2_500_000_000)))
        gui.motor6_pending = gui.motor6_lease = gui.operation_conveyor_request = None
        gui.operation_conveyor_target = None
        gui.motor6_running = False
        gui.motor6_command_rpm = 0.0
        gui.motor6_entry = Value("75.398")
        gui.motor6_accel_entry = gui.motor6_decel_entry = Value("96.0")
        gui.status_text = Value()
        gui.motor6_index_feedback = Value()
        gui.motor6_pitch_text = Value()
        gui.operation_entries = {"Step 1 · Motor 6 positioning distance (mm)": Value("140.825")}
        gui.operation_step_index = 0
        gui._update_operation_step_text = lambda: None

    def test_single_step_request_and_lease_instead_of_gui_braking(self):
        self.assertTrue(self.gui.step_motor6(1))
        self.assertEqual(self.sent[0]["index"], 1)
        self.assertAlmostEqual(self.sent[0]["rpm"], 48.0, places=3)
        self.gui._update_motor6_index()
        self.assertEqual(self.heartbeats, [self.sent[0]["id"]])
        self.assertFalse(self.gui.step_motor6(1))
        self.assertEqual(len(self.sent), 1)

    def test_rejection_does_not_advance_operation(self):
        self.gui.operation_next()
        self.gui.node.motor6_index_status.update(request_id=self.sent[0]["id"],
                                                request_ok=False, request_message="not ready")
        self.gui._update_motor6_index()
        self.assertEqual(self.gui.operation_step_index, 0)
        self.assertIsNone(self.gui.motor6_lease)

    def test_sequence_waits_for_settled_matching_target(self):
        self.gui.operation_next()
        status = self.gui.node.motor6_index_status
        status.update(request_id=self.sent[0]["id"], request_ok=True, state="indexing",
                      reference_id=1, target_index=1, stopped=False)
        self.gui._update_motor6_index()
        self.assertEqual(self.gui.operation_step_index, 1)
        self.assertFalse(self.gui._operation_conveyor_ready())
        status.update(state="ready", stopped=True, index=1)
        self.assertTrue(self.gui._operation_conveyor_ready())
        status.update(reference_id=2)
        self.assertFalse(self.gui._operation_conveyor_ready())

    def test_status_loss_and_unacknowledged_request_stop(self):
        self.gui.step_motor6(1)
        self.gui.node.motor6_status_fresh = lambda: False
        self.gui._update_motor6_index()
        self.assertEqual(self.stops, [0.0])
        self.assertIsNone(self.gui.motor6_lease)
        self.gui.node.motor6_status_fresh = lambda: True
        self.gui.step_motor6(1)
        self.now += 1.01
        self.gui._update_motor6_index()
        self.assertEqual(self.stops, [0.0, 0.0])
        self.assertIsNone(self.gui.motor6_pending)


if __name__ == "__main__":
    unittest.main()
