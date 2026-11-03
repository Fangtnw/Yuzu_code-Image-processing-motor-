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
from motor_controller.conveyor_index import ConveyorIndex, COUNTS_PER_RAD, OPERATING_MAX_RPM, MM_S_PER_RPM  # noqa: E402


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
                           Float64MultiArray=NS, Float64=NS, Float32=NS,
                           Float32MultiArray=NS, Empty=NS, String=NS)
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
        g.startup_position_counts = None
        g.startup_stable_since = None
        g.last_stop_stamp = 0
        g.feedback_joint = "motor1_motor2"
        g.velocity_interface = "motor6_velocity"
        g.position_interface = "motor6_position"
        g.publisher = Publisher()
        g.status_publisher = Publisher()
        g.external_status_publisher = Publisher()
        self.errors = []
        g.get_logger = lambda: NS(error=self.errors.append)
        g.get_clock = lambda: NS(now=lambda: NS(nanoseconds=self.stamp()))

    def stamp(self):
        return 2_000_000_000 + round(self.now * 1e9)

    def request(self, **overrides):
        data = dict(id="request-1", session=self.guard.session,
                    reference_id=self.guard.motion.reference_id, op="index", index=1,
                    rpm=OPERATING_MAX_RPM, acceleration=120.0, deceleration=120.0, sent_ns=self.stamp())
        data.update(overrides)
        return NS(data=json.dumps(data))

    def feedback_message(self, names=None, values=None, stamp=None):
        stamp = self.stamp() if stamp is None else stamp
        return NS(header=NS(stamp=NS(sec=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000)),
                  joint_names=[self.guard.feedback_joint], interface_values=[NS(
                      interface_names=names or [self.guard.position_interface, self.guard.velocity_interface, "motor6_status"],
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
        self.guard.on_index_request(request)
        self.assertEqual(self.guard.motion.last_command, self.now)
        self.now += .01
        self.guard.on_heartbeat(NS(data="wrong-owner"))
        self.assertNotEqual(self.guard.motion.last_command, self.now)
        self.guard.on_heartbeat(NS(data="request-1"))
        self.assertEqual(self.guard.motion.last_command, self.now)
        self.guard.on_index_request(self.request(id="duplicate-index"))
        self.assertEqual(self.guard.lease, "request-1")
        self.guard.on_command(NS(data=[0.0]))
        self.now += .01
        self.guard.on_index_request(request)
        self.assertEqual(self.guard.motion.state, "interrupted")
        self.assertIsNone(self.guard.lease)
        self.assertNotEqual(self.guard.motion.last_command, self.now)

    def test_external_stop_and_status_alias(self):
        self.guard.motion.state = "indexing"
        self.guard.on_external_stop(NS())
        self.assertEqual(self.guard.motion.state, "interrupted")
        self.assertEqual(
            self.guard.status_publisher.messages[-1].data,
            self.guard.external_status_publisher.messages[-1].data,
        )

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
        for joint, position, velocity in (("motor1_motor2", "motor6_position", "motor6_velocity"), ("motor6_joint", "position", "velocity")):
            self.guard.motion = ConveyorIndex()
            self.guard.last_feedback_stamp = None
            self.guard.startup_position_counts = None
            self.guard.startup_stable_since = None
            self.guard.feedback_joint, self.guard.position_interface, self.guard.velocity_interface = joint, position, velocity
            for _ in range(65):
                self.now += .005
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


class ExternalCommandTests(unittest.TestCase):
    def test_motor2_gui_accepts_90_and_100_percent_output_speed(self):
        module = load_without_ros("yuzu_peeler_gui.py")
        for rpm in (374.4, 416.0):
            self.assertEqual(module.checked_axis2_rpm(rpm), rpm)
            self.assertEqual(module.checked_rotary_ramp(832.0, 832.0, 832.0, rpm), (832.0, 832.0))
        with self.assertRaises(ValueError):
            module.checked_axis2_rpm(416.1)

    def test_gui_uses_scalar_targets_and_empty_stop(self):
        module = load_without_ros("yuzu_peeler_gui.py")
        node = module.YuzuOperatorNode.__new__(module.YuzuOperatorNode)
        node.motor1_publisher = Publisher()
        node.motor2_publisher = Publisher()
        node.motor6_publisher = Publisher()
        node.publish_axis1(0.12)
        node.publish_axis2(100.0)
        node.publish_motor6(0.0)
        self.assertEqual(node.motor1_publisher.messages[-1].data, 120.0)
        self.assertEqual(node.motor2_publisher.messages[-1].data, 100.0)
        self.assertEqual(len(node.motor6_publisher.messages), 1)
        self.assertFalse(hasattr(node.motor6_publisher.messages[-1], "data"))

    def test_motor1_millimetres_use_existing_guard(self):
        module = load_without_ros("azd3a_axis1_command_guard.py")
        guard = module.Axis1CommandGuard.__new__(module.Axis1CommandGuard)
        guard.min_position = 0.0
        guard.max_position = 0.4
        guard.command_position = 0.1
        guard.operator_command_received = False
        guard.get_logger = lambda: NS(info=lambda message: None, error=lambda message: None)
        guard.on_external_command(NS(data=120.0))
        self.assertAlmostEqual(guard.target_position, 0.12)
        self.assertTrue(guard.operator_command_received)
        guard.on_external_command(NS(data=500.0))
        self.assertAlmostEqual(guard.target_position, 0.12)

    def test_motor2_scalar_uses_existing_watchdog_guard(self):
        module = load_without_ros("azd3a_axis2_velocity_guard.py")
        guard = module.Axis2VelocityGuard.__new__(module.Axis2VelocityGuard)
        guard.max_rpm = 416.0
        guard.max_acceleration_rpm_s = 832.0
        guard.acceleration_rpm_s = 832.0
        guard.deceleration_rpm_s = 832.0
        guard.output_rpm = 0.0
        guard.get_clock = lambda: NS(now=lambda: "stamp")
        guard.get_logger = lambda: NS(error=lambda message: None)
        guard.on_external_command(NS(data=100.0))
        self.assertEqual(guard.target_rpm, 100.0)
        self.assertEqual(guard.last_command_time, "stamp")
        guard.on_external_command(NS(data=374.4))
        self.assertEqual(guard.target_rpm, 374.4)
        guard.on_external_command(NS(data=416.0))
        self.assertEqual(guard.target_rpm, 416.0)
        guard.on_external_command(NS(data=416.1))
        self.assertEqual(guard.target_rpm, 416.0)

    def test_motor5_zero_is_published_for_remote_gui(self):
        module = load_without_ros("azd3a_motor5_position_guard.py")
        guard = module.Motor5PositionGuard.__new__(module.Motor5PositionGuard)
        guard.measured_position = 1.25
        guard.command_velocity = 0.0
        guard.zero_publisher = Publisher()
        guard.get_logger = lambda: NS(info=lambda message: None, error=lambda message: None)
        guard.on_set_zero(NS())
        self.assertEqual(guard.zero_publisher.messages[-1].data, 1.25)


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
        self.sent, self.stops = [], []
        gui.node = NS(motor6_index_status={"session": "test", "reference_id": 0, "index": 0,
                                          "referenced": False, "state": "unreferenced",
                                          "stopped": True, "fresh": True, "pitch_mm": 140.0},
                      motor6_status_fresh=lambda: True, motor6_index_publisher=Publisher(),
                      publish_motor6_index=self.sent.append,
                      publish_motor6=self.stops.append,
                      axis1_position_m=0.0,
                      feedback_is_fresh_for=lambda motor: True,
                      drive_status={f"motor{motor}": 0x0027 for motor in range(1, 7)},
                      get_clock=lambda: NS(now=lambda: NS(nanoseconds=2_500_000_000)))
        gui.motor6_pending = gui.motor6_lease = gui.operation_conveyor_request = None
        gui.motor6_lease_request = None
        gui.operation_conveyor_target = None
        gui.operation_process = None
        gui.operation_contour_file = None
        gui.operation_pending_advance = False
        gui.operation_auto_running = False
        gui.operation_capture_after = None
        gui.operation_capture_deadline = None
        gui.operation_conveyor_deadline = None
        gui.motor6_running = False
        gui.motor6_command_rpm = 0.0
        gui.motor6_entry = Value(f"{self.module.MOTOR6_GUI_MAX_MM_S:.3f}")
        gui.motor6_accel_entry = gui.motor6_decel_entry = Value("120.0")
        gui.status_text = Value()
        gui.motor6_index_feedback = Value()
        gui.motor6_pitch_text = Value()
        gui.operation_entries = {
            "Step 1 · Motor 6 positioning distance (mm)": Value("140.000"),
            "Step 1 · Conveyor speed (mm/s)": Value("100.000"),
            "Step 1 · Conveyor accel (rpm/s)": Value("120.0"),
            "Step 1 · Conveyor decel (rpm/s)": Value("120.0"),
        }
        gui.operation_step_index = 0
        gui._update_operation_step_text = lambda: None

    def test_single_step_request_and_lease_instead_of_gui_braking(self):
        self.assertTrue(self.gui.step_motor6(1))
        self.assertEqual(self.sent[0]["index"], 1)
        self.assertAlmostEqual(self.sent[0]["rpm"], OPERATING_MAX_RPM, places=3)
        self.gui._update_motor6_index()
        self.assertEqual(self.sent[1], self.sent[0])
        self.assertFalse(self.gui.step_motor6(1))
        self.assertEqual(len(self.sent), 2)

    def test_resume_request_uses_current_lower_gui_speed(self):
        self.gui.motor6_entry = Value("100.000")
        self.gui.resume_motor6()
        self.assertEqual(self.sent[0]["op"], "resume")
        self.assertAlmostEqual(self.sent[0]["rpm"], 100.0 / MM_S_PER_RPM)
        self.assertEqual(self.sent[0]["acceleration"], 120.0)
        self.assertEqual(self.sent[0]["deceleration"], 120.0)

    def test_rejection_does_not_advance_operation(self):
        self.gui.operation_next()
        self.assertAlmostEqual(self.sent[0]["rpm"], 100.0 / MM_S_PER_RPM)
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

    def test_fixed_snapshot_is_reused_for_motor1_lift_and_contour_peel(self):
        gui = self.gui
        gui.operation_data_source = Value("Fixed demo values")
        gui.operation_snapshot_text = Value()
        gui.operation_snapshot = None
        gui.operation_contour_file = None
        gui.operation_steps = ("conveyor", "capture", "pierce", "lift", "peel", "home")
        gui.operation_entries.update({
            "Camera frame Motor 1 encoder (mm)": Value("204"),
            "Motor 1 home (mm)": Value("0"),
            "Motor 1 lift distance (mm)": Value("150"),
            "Motor 1 extra allowance (mm)": Value("0"),
            "Fixed demo Y offset (mm)": Value("-30"),
            "Fixed demo inset radius (mm)": Value("17"),
            "Motor 3 center distance (mm)": Value("35"),
            "Motor 4 center distance (mm)": Value("35"),
            "Motor 5 sweep speed (deg/s)": Value("10"),
        })
        commands = []
        gui._operation_start_process = lambda command, step, advance: commands.append((command, step, advance))
        gui._operation_run_step(1, False)
        self.assertEqual(gui.operation_snapshot["offset"], -30.0)
        self.assertEqual(len(gui.operation_snapshot["contour"]), 216)
        gui._operation_run_step(2, False)
        self.assertIn("insert", commands[-1][0])
        gui.node.axis1_position_m = 0.234
        gui._operation_run_step(3, False)
        self.assertIn("lift", commands[-1][0])
        self.assertIn("-30.0", commands[-1][0])
        gui._operation_run_step(4, False)
        self.assertIn("--contour-file", commands[-1][0])
        self.assertEqual(commands[-1][1:], (4, False))
        import os
        os.unlink(gui.operation_contour_file)

    def test_camera_snapshot_rejects_stale_sample_and_keeps_captured_values(self):
        gui = self.gui
        gui.operation_data_source = Value("Camera snapshot")
        gui.operation_snapshot_text = Value()
        gui.operation_snapshot = None
        gui.operation_entries.update({
            "Camera frame Motor 1 encoder (mm)": Value("204"),
            "Motor 1 home (mm)": Value("0"),
            "Motor 1 lift distance (mm)": Value("150"),
            "Motor 1 extra allowance (mm)": Value("0"),
            "Motor 3 center distance (mm)": Value("35"),
            "Motor 4 center distance (mm)": Value("35"),
            "Motor 5 sweep speed (deg/s)": Value("10"),
        })
        import math
        contour = [v for a in range(0, 360, 5)
                   for v in (a, 17 * math.sin(math.radians(a)),
                             17 * math.cos(math.radians(a)))]
        gui.node.camera_offset = -30.0
        gui.node.camera_contour = contour
        gui.node.camera_offset_at = self.now - 2.0
        gui.node.camera_contour_at = self.now
        self.assertFalse(gui.operation_capture_snapshot())
        gui.node.camera_offset_at = self.now
        self.assertTrue(gui.operation_capture_snapshot())
        gui.node.camera_offset = 5.0
        gui.node.camera_contour = []
        self.assertEqual(gui.operation_snapshot["offset"], -30.0)
        self.assertEqual(len(gui.operation_snapshot["contour"]), 216)

    def test_selected_step_success_sets_next_sequence_step(self):
        gui = self.gui
        gui.operation_steps = ("conveyor", "capture", "pierce", "lift", "peel", "home")
        gui.operation_process = NS(poll=lambda: 0,
                                   communicate=lambda: ("Motor 1 reached 232.9020 mm", ""),
                                   returncode=0)
        gui.operation_process_step = (2, False)
        gui.operation_contour_file = None
        gui.operation_step_index = 0
        gui._poll_operation_process()
        self.assertEqual(gui.operation_step_index, 3)
        self.assertIn("pierce", gui.status_text.get())

    def test_pierce_from_nonhome_reports_measured_start_and_sends_no_command(self):
        gui = self.gui
        gui.operation_process = None
        gui.operation_snapshot = {"offset": -28.902,
                                  "motor1_reference": self.module.Motor1CameraReference(),
                                  "lift_mm": 150.0, "contour": ()}
        gui.node.axis1_position_m = 0.0293532
        commands = []
        gui._operation_start_process = lambda *args: commands.append(args)
        gui._operation_run_step(2, False)
        self.assertEqual(commands, [])
        self.assertIn("29.353", self.errors[-1])

    def test_full_sequence_waits_for_conveyor_stop_then_new_camera_snapshot(self):
        gui = self.gui
        gui.operation_steps = ("conveyor", "capture", "pierce", "lift", "peel", "home")
        gui.operation_snapshot = None
        gui.operation_snapshot_text = Value()
        gui.operation_data_source = Value("Camera snapshot")
        gui.operation_entries.update({
            "Camera frame Motor 1 encoder (mm)": Value("204"),
            "Motor 1 home (mm)": Value("0"),
            "Motor 1 lift distance (mm)": Value("150"),
            "Motor 1 extra allowance (mm)": Value("0"),
            "Motor 3 center distance (mm)": Value("35"),
            "Motor 4 center distance (mm)": Value("35"),
            "Motor 5 sweep speed (deg/s)": Value("10"),
        })
        gui.operation_auto_running = True
        gui.operation_conveyor_deadline = self.now + 30.0
        gui.operation_step_index = 1
        gui.operation_conveyor_target = ("test", 1, 1)
        status = gui.node.motor6_index_status
        status.update(state="indexing", stopped=False, reference_id=1, target_index=1)
        gui._advance_full_sequence()
        self.assertIsNone(gui.operation_capture_after)
        status.update(state="ready", stopped=True, index=1)
        gui._advance_full_sequence()
        self.assertEqual(gui.operation_capture_after, self.now)
        import math
        gui.node.camera_offset = -30.0
        gui.node.camera_contour = [v for a in range(0, 360, 5)
                                   for v in (a, 17 * math.sin(math.radians(a)),
                                             17 * math.cos(math.radians(a)))]
        gui.node.camera_offset_at = self.now - 0.1
        gui.node.camera_contour_at = self.now - 0.1
        gui._advance_full_sequence()
        self.assertIsNone(gui.operation_snapshot)
        self.now += 0.1
        gui.node.camera_offset_at = self.now
        gui.node.camera_contour_at = self.now
        gui._advance_full_sequence()
        self.assertEqual(gui.operation_step_index, 2)
        self.assertEqual(gui.operation_snapshot["offset"], -30.0)
        commands = []
        def start_fake_step(step, advance):
            commands.append((step, advance))
            gui.operation_process = NS(poll=lambda: 0, communicate=lambda: ("reached", ""), returncode=0)
            gui.operation_process_step = (step, advance)
        gui._operation_run_step = start_fake_step
        for step in (2, 3, 4, 5):
            gui._advance_full_sequence()
            self.assertEqual(commands[-1], (step, True))
            gui._poll_operation_process()
        gui._advance_full_sequence()
        self.assertEqual(gui.operation_step_index, 6)
        self.assertFalse(gui.operation_auto_running)

    def test_full_sequence_button_starts_with_only_conveyor_index(self):
        gui = self.gui
        gui.operation_steps = ("conveyor", "capture", "pierce", "lift", "peel", "home")
        gui.operation_snapshot = {"old": True}
        gui.operation_snapshot_text = Value()
        gui.operation_entries.update({
            "Camera frame Motor 1 encoder (mm)": Value("204"),
            "Motor 1 home (mm)": Value("0"),
            "Motor 1 lift distance (mm)": Value("150"),
            "Motor 1 extra allowance (mm)": Value("0"),
        })
        gui.node.motor3_position_m = 0.0
        gui.node.motor4_position_m = 0.0
        gui.node.motor5_position_rad = 1.5
        gui.node.motor5_zero_position_rad = 1.5
        gui.operation_run_full()
        self.assertTrue(gui.operation_auto_running)
        self.assertEqual(gui.operation_step_index, 0)
        self.assertIsNone(gui.operation_snapshot)
        self.assertEqual(len(self.sent), 1)
        self.assertIsNotNone(gui.operation_conveyor_request)


if __name__ == "__main__":
    unittest.main()
