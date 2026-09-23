import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
MOTION_CMD = REPO_ROOT / "motion_cmd"


class Azd3aEthercatConfigFileTests(unittest.TestCase):
    def test_slave_yaml_matches_azd_ked_identity_and_required_cia402_pdos(self):
        config = MOTION_CMD / "config" / "azd3a_ked_cia402_slave.yaml"
        text = config.read_text(encoding="utf-8")

        self.assertIn("vendor_id: 0x000002BE", text)
        self.assertIn("product_id: 0x000013AF", text)
        self.assertIn("revision 0x01110301", text)
        self.assertRegex(text.lower(), r"rpdo:\s*\n\s*-\s*index:\s*0x1600")
        self.assertRegex(text.lower(), r"tpdo:\s*\n\s*-\s*index:\s*0x1a00")
        for index in ("0x6040", "0x607a", "0x6060"):
            self.assertRegex(text.lower(), rf"index:\s*{index}")
        for index in ("0x6041", "0x6064", "0x6061"):
            self.assertRegex(text.lower(), rf"index:\s*{index}")
        self.assertNotRegex(text.lower(), r"index:\s*0x60ff")
        self.assertNotRegex(text.lower(), r"index:\s*0x60b8")
        self.assertIn("factor: -833333.3333333334, offset: 1999.0", text)
        self.assertIn("factor: -0.0000012, offset: 0.0023988", text)
        self.assertIn("assign_activate: 0x0300", text)
        self.assertIn("auto_fault_reset: false", text)

    def test_feedback_launch_cannot_auto_enable_cia402_or_load_motion_controller(self):
        xacro = MOTION_CMD / "urdf" / "azd3a_axis1_feedback.urdf.xacro"
        launch = MOTION_CMD / "launch" / "azd3a_axis1_feedback.launch.py"
        xacro_text = xacro.read_text(encoding="utf-8")
        launch_text = launch.read_text(encoding="utf-8")

        self.assertIn("ethercat_generic_plugins/GenericEcSlave", xacro_text)
        self.assertNotIn("ethercat_generic_plugins/EcCiA402Drive", xacro_text)
        self.assertIn('name="axis1_joint" type="prismatic"', xacro_text)
        self.assertIn(
            'lower="0.000" upper="0.400" velocity="0.600"', xacro_text
        )
        self.assertIn('<param name="min">0.0000996</param>', xacro_text)
        self.assertIn('<param name="max">0.200</param>', xacro_text)
        self.assertIn('<param name="control_frequency">200</param>', xacro_text)
        self.assertIn("joint_state_broadcaster", launch_text)
        self.assertIn("ParameterValue", launch_text)
        self.assertIn("value_type=str", launch_text)
        self.assertNotIn("trajectory", launch_text)
        self.assertNotIn("forward_command", launch_text)

    def test_axis2_feedback_is_read_only_and_uses_live_output_scaling(self):
        slave = MOTION_CMD / "config" / "azd3a_axis2_feedback_slave.yaml"
        xacro = MOTION_CMD / "urdf" / "azd3a_axis2_feedback.urdf.xacro"
        launch = MOTION_CMD / "launch" / "azd3a_axis2_feedback.launch.py"
        slave_text = slave.read_text(encoding="utf-8").lower()
        xacro_text = xacro.read_text(encoding="utf-8")
        launch_text = launch.read_text(encoding="utf-8")

        self.assertRegex(slave_text, r"rpdo:\s*\n\s*-\s*index:\s*0x1610")
        self.assertRegex(slave_text, r"tpdo:\s*\n\s*-\s*index:\s*0x1a11")
        for index in ("0x6841", "0x6864", "0x686c", "0x6861"):
            self.assertRegex(slave_text, rf"index:\s*{index}")
        self.assertEqual(slave_text.count("factor: 0.00008726646259971647"), 2)
        self.assertIn("auto_fault_reset: false", slave_text)
        self.assertNotIn("command_interface:", slave_text)
        self.assertIn('name="axis2_joint" type="continuous"', xacro_text)
        self.assertIn('<state_interface name="position"/>', xacro_text)
        self.assertIn('<state_interface name="velocity"/>', xacro_text)
        self.assertNotIn("<command_interface", xacro_text)
        self.assertIn("GenericEcSlave", xacro_text)
        self.assertIn("joint_state_broadcaster", launch_text)

    def test_axis2_spin_has_commissioning_cap_ramp_and_watchdog(self):
        slave = MOTION_CMD / "config" / "azd3a_axis2_tiny_spin_slave.yaml"
        xacro = MOTION_CMD / "urdf" / "azd3a_axis2_tiny_spin.urdf.xacro"
        guard = MOTION_CMD / "motor_controller" / "azd3a_axis2_velocity_guard.py"
        launch = MOTION_CMD / "launch" / "azd3a_axis2_tiny_spin.launch.py"
        slave_text = slave.read_text(encoding="utf-8").lower()
        xacro_text = xacro.read_text(encoding="utf-8")
        guard_text = guard.read_text(encoding="utf-8")
        launch_text = launch.read_text(encoding="utf-8")

        self.assertRegex(slave_text, r"rpdo:\s*\n\s*-\s*index:\s*0x1612")
        self.assertRegex(slave_text, r"tpdo:\s*\n\s*-\s*index:\s*0x1a11")
        self.assertIn("command_interface: velocity", slave_text)
        self.assertIn("factor: 11459.155902616465", slave_text)
        self.assertIn("default: 0", slave_text)
        self.assertIn("mode_of_operation\">9", xacro_text)
        self.assertIn("cia402_object_index_offset\">0x0800", xacro_text)
        self.assertIn("26.179938779914945", xacro_text)
        self.assertIn('DeclareLaunchArgument(\n                "max_rpm"', launch_text)
        self.assertIn('default_value="250.0"', launch_text)
        self.assertIn('"max_acceleration_rpm_s",\n                default_value="500.0"', launch_text)
        self.assertIn('LaunchConfiguration("max_rpm")', launch_text)
        for expected in (
            "HARD_MAX_RPM = 416.0",
            "COMMISSIONING_MAX_RPM = 250.0",
            "DEFAULT_ACCELERATION_RPM_S = 10.0",
            "COMMAND_TIMEOUT_S = 0.5",
            'PUBLIC_TOPIC = "/axis2_velocity_controller/commands_rpm"',
            'RAW_TOPIC = "/axis2_raw_velocity_controller/commands"',
            "len(message.data) not in (1, 3)",
        ):
            self.assertIn(expected, guard_text)

    def test_axis3_feedback_is_read_only_and_uses_reference_output_scaling(self):
        slave = MOTION_CMD / "config" / "azd3a_axis3_feedback_slave.yaml"
        xacro = MOTION_CMD / "urdf" / "azd3a_axis3_feedback.urdf.xacro"
        launch = MOTION_CMD / "launch" / "azd3a_axis3_feedback.launch.py"
        slave_text = slave.read_text(encoding="utf-8").lower()
        xacro_text = xacro.read_text(encoding="utf-8")
        launch_text = launch.read_text(encoding="utf-8")

        self.assertRegex(slave_text, r"rpdo:\s*\n\s*-\s*index:\s*0x1620")
        self.assertRegex(slave_text, r"tpdo:\s*\n\s*-\s*index:\s*0x1a21")
        for index in ("0x7041", "0x7064", "0x706c", "0x7061"):
            self.assertRegex(slave_text, rf"index:\s*{index}")
        self.assertEqual(slave_text.count("factor: 0.00003141592653589793"), 2)
        self.assertIn("auto_fault_reset: false", slave_text)
        self.assertNotIn("command_interface:", slave_text)
        self.assertIn('name="axis3_joint" type="continuous"', xacro_text)
        self.assertNotIn("<command_interface", xacro_text)
        self.assertIn("GenericEcSlave", xacro_text)
        self.assertIn("joint_state_broadcaster", launch_text)

    def test_axis3_spin_has_requirement_cap_ramp_and_watchdog(self):
        slave_text = (MOTION_CMD / "config" / "azd3a_axis3_tiny_spin_slave.yaml").read_text(encoding="utf-8").lower()
        xacro_text = (MOTION_CMD / "urdf" / "azd3a_axis3_tiny_spin.urdf.xacro").read_text(encoding="utf-8")
        guard_text = (MOTION_CMD / "motor_controller" / "azd3a_axis3_velocity_guard.py").read_text(encoding="utf-8")

        self.assertRegex(slave_text, r"rpdo:\s*\n\s*-\s*index:\s*0x1622")
        self.assertRegex(slave_text, r"tpdo:\s*\n\s*-\s*index:\s*0x1a21")
        self.assertIn("factor: 31830.98861837907", slave_text)
        self.assertIn("mode_of_operation\">9", xacro_text)
        self.assertIn("cia402_object_index_offset\">0x1000", xacro_text)
        self.assertIn("2.0943951023931953", xacro_text)
        for expected in (
            "HARD_MAX_RPM = 150.0",
            "COMMISSIONING_MAX_RPM = 5.0",
            "DEFAULT_ACCELERATION_RPM_S = 2.0",
            'PUBLIC_TOPIC = "/axis3_velocity_controller/commands_rpm"',
            'RAW_TOPIC = "/axis3_raw_velocity_controller/commands"',
        ):
            self.assertIn(expected, guard_text)

    def test_axis3_index_is_csp_feedback_synchronized_and_bounded(self):
        slave_text = (MOTION_CMD / "config" / "azd3a_axis3_index_slave.yaml").read_text(encoding="utf-8").lower()
        xacro_text = (MOTION_CMD / "urdf" / "azd3a_axis3_index.urdf.xacro").read_text(encoding="utf-8")
        guard_text = (MOTION_CMD / "motor_controller" / "azd3a_axis3_index_guard.py").read_text(encoding="utf-8")
        self.assertRegex(slave_text, r"rpdo:\s*\n\s*-\s*index:\s*0x1620")
        self.assertRegex(slave_text, r"tpdo:\s*\n\s*-\s*index:\s*0x1a21")
        self.assertIn("command_interface: position", slave_text)
        self.assertIn("default: .nan", slave_text)
        self.assertIn("cia402_object_index_offset\">0x1000", xacro_text)
        self.assertIn("position_startup_tolerance\">0.000032", xacro_text)
        self.assertIn("mode_of_operation\">8", xacro_text)
        self.assertIn('PUBLIC_TOPIC = "/axis3_position_controller/commands_deg"', guard_text)
        self.assertIn("0.0 <= offset_deg <= 90.0", guard_text)
        self.assertIn("self.origin + math.radians(offset_deg)", guard_text)
        self.assertIn("HARD_MAX_ACCELERATION_RPM_S = 60.0", guard_text)

    def test_python_check_controller_checks_igh_surfaces_only(self):
        script = MOTION_CMD / "motor_controller" / "azd3a_ethercat_check.py"
        text = script.read_text(encoding="utf-8")

        for command in (
            '["ethercat", "slaves"]',
            '["ethercat", "pdos"]',
            '["ethercat", "sdos"]',
        ):
            self.assertIn(command, text)
        self.assertIn("--dry-run", text)
        self.assertIn('["ros2", "pkg", "prefix", "ethercat_generic_slave"]', text)
        self.assertIn(
            '["ros2", "pkg", "prefix", "ethercat_generic_cia402_drive"]',
            text,
        )
        self.assertNotIn('["ros2", "pkg", "prefix", "ethercat_generic_plugins"]', text)
        self.assertNotIn("robot_description", text)
        self.assertNotIn("ros2_control_node", text)
        self.assertNotIn("ros2 control", text)

    def test_setup_exposes_python_check_command(self):
        setup_py = MOTION_CMD / "setup.py"
        text = setup_py.read_text(encoding="utf-8")

        self.assertIn("azd3a_ethercat_check", text)
        self.assertIn("yuzu_operator_gui", text)
        self.assertNotIn("scripts/*.sh", text)

    def test_operator_gui_uses_only_guarded_topics_and_safe_limits(self):
        gui = MOTION_CMD / "motor_controller" / "yuzu_operator_gui.py"
        text = gui.read_text(encoding="utf-8")

        self.assertIn('AXIS1_TOPIC = "/axis1_position_controller/commands"', text)
        self.assertIn('AXIS2_TOPIC = "/axis2_velocity_controller/commands_rpm"', text)
        self.assertIn('MOTOR3_TOPIC = "/motor3_position_controller/commands_mm"', text)
        self.assertIn('MOTOR4_TOPIC = "/motor4_position_controller/commands_mm"', text)
        self.assertIn('MOTOR5_TOPIC = "/motor5_position_controller/commands_deg"', text)
        self.assertIn('MOTOR5_SET_ZERO_TOPIC = "/motor5_position_controller/set_zero"', text)
        self.assertIn('MOTOR5_MANUAL_STEP_TOPIC = "/motor5_position_controller/manual_step_deg"', text)
        self.assertIn('MOTOR6_TOPIC = "/motor6_conveyor/commands_rpm"', text)
        self.assertNotIn("/axis1_raw_position_controller/commands", text)
        self.assertNotIn("/axis2_raw_velocity_controller/commands", text)
        self.assertIn("AXIS1_MIN_MM = 0.0", text)
        self.assertIn("AXIS1_MAX_MM = 400.0", text)
        self.assertIn("AXIS1_INCREMENT_MM = 0.0012", text)
        self.assertIn("AXIS1_VELOCITY_MM_S = 15.0", text)
        self.assertIn("AXIS1_ACCELERATION_MM_S2 = 15.0", text)
        self.assertIn("AXIS2_GUI_MAX_RPM = 250.0", text)
        self.assertIn("AXIS2_ACCELERATION_RPM_S = 500.0", text)
        self.assertIn("MOTOR4_MIN_MM = 0.0", text)
        self.assertIn("MOTOR4_MAX_MM = 20.0", text)
        self.assertIn("MOTOR3_MAX_MM = 20.0", text)
        self.assertIn("MOTOR4_INCREMENT_MM = 0.0001", text)
        self.assertIn("MOTOR3_4_VELOCITY_MM_S = 8.0", text)
        self.assertIn("MOTOR3_ACCELERATION_MM_S2 = 50.0", text)
        self.assertIn("MOTOR4_ACCELERATION_MM_S2 = 50.0", text)
        self.assertIn("MOTOR5_MAX_ANGLE_DEG = 180.0", text)
        self.assertIn("MOTOR5_HARDWARE_STEP_DEG = 0.0018", text)
        self.assertIn("Set current position as zero", text)
        self.assertIn("CW (+)", text)
        self.assertIn("CCW (−)", text)
        self.assertIn("step_target", text)
        self.assertIn("step_axis1", text)
        self.assertIn("step_motor3", text)
        self.assertIn("step_motor4", text)
        self.assertIn("step_motor5", text)
        self.assertIn("publish_motor5_manual_step", text)
        self.assertIn('text=label', text)
        self.assertIn("if self.motor5_origin_rad is None:", text)
        self.assertIn("MOTOR6_GUI_MAX_RPM = 60.0", text)
        self.assertIn("MOTOR6_PULLEY_DIAMETER_MM = 30.0", text)
        self.assertIn("checked_motor6_speed_mm_s", text)
        self.assertIn("get_subscription_count()", text)
        self.assertIn("self.root.after(100, self.tick)", text)
        self.assertIn("Show motion details", text)
        self.assertIn("Step 1 · Motor 6 positioning distance (mm)", text)
        self.assertIn("Step 4.1 · Motor 2 CCW speed (rpm)", text)
        self.assertIn("Step 6 · Motor 1 home position (mm)", text)
        self.assertIn("Acceleration", text)
        self.assertIn("Deceleration", text)
        self.assertIn("aligned_motor3_position_mm", text)
        self.assertIn("Move Motor 3", text)
        self.assertIn('DynamicJointState, "/dynamic_joint_states"', text)

    def test_combined_backend_uses_all_three_slave0_axes(self):
        slave_text = (MOTION_CMD / "config" / "azd3a_motor1_motor2_slave.yaml").read_text(encoding="utf-8").lower()
        xacro_text = (MOTION_CMD / "urdf" / "azd3a_motor1_motor2.urdf.xacro").read_text(encoding="utf-8")
        controller_text = (MOTION_CMD / "config" / "azd3a_motor1_motor2_controllers.yaml").read_text(encoding="utf-8")
        launch_text = (MOTION_CMD / "launch" / "azd3a_motor1_motor2_gui.launch.py").read_text(encoding="utf-8")

        for pdo in ("0x1600", "0x1612", "0x1620", "0x1a00", "0x1a11", "0x1a20"):
            self.assertRegex(slave_text, rf"index:\s*{pdo}")
        self.assertIn("command_interface: position", slave_text)
        self.assertIn("command_interface: velocity", slave_text)
        self.assertIn("command_interface: motor3_position", slave_text)
        self.assertIn('<param name="number_of_physical_drives">2</param>', xacro_text)
        self.assertIn('<param name="secondary_cia402_object_index_offset">0x0800</param>', xacro_text)
        self.assertIn('<param name="secondary_mode_of_operation">9</param>', xacro_text)
        self.assertIn('<param name="tertiary_cia402_object_index_offset">0x1000</param>', xacro_text)
        self.assertIn('<param name="tertiary_mode_of_operation">8</param>', xacro_text)
        control_block = xacro_text.split('<ros2_control', 1)[1]
        motor1_block, motor4_block = control_block.split('<joint name="motor4_joint">', 1)
        self.assertNotIn('name="motor5_position"', motor1_block)
        self.assertIn('<command_interface name="motor5_position"/>', motor4_block)
        self.assertIn('<state_interface name="motor5_position"/>', motor4_block)
        self.assertIn('<state_interface name="motor6_position"/>', motor4_block)
        self.assertIn(
            '<param name="tertiary_cia402_object_index_offset">0x0800</param>',
            motor4_block,
        )
        self.assertIn('<param name="position">1</param>', xacro_text)
        self.assertIn("azd3a_motor4_motor6_slave.yaml", xacro_text)
        self.assertIn('<param name="secondary_cia402_object_index_offset">0x1000</param>', xacro_text)
        self.assertIn('<param name="secondary_mode_of_operation">9</param>', xacro_text)
        self.assertEqual(xacro_text.count("position_startup_tolerance"), 1)
        self.assertIn("axis1_raw_position_controller", controller_text)
        self.assertIn("axis2_raw_velocity_controller", controller_text)
        self.assertIn("motor3_raw_position_controller", controller_text)
        self.assertIn("motor4_raw_position_controller", controller_text)
        self.assertIn("motor6_raw_velocity_controller", controller_text)
        self.assertIn('"max_rpm": 250.0', launch_text)
        self.assertIn('"max_acceleration_rpm_s": 500.0', launch_text)
        self.assertIn('executable="azd3a_motor4_position_guard"', launch_text)
        self.assertIn('executable="azd3a_motor3_position_guard"', launch_text)
        self.assertIn('executable="azd3a_motor6_conveyor_guard"', launch_text)
        self.assertIn('"max_rpm": 60.0', launch_text)
        self.assertIn('"max_acceleration_rpm_s": 250.0', launch_text)
        self.assertIn('executable="yuzu_operator_gui"', launch_text)

    def test_motor3_combined_guard_is_absolute_feedback_synchronized_and_bounded(self):
        guard_text = (
            MOTION_CMD / "motor_controller" / "azd3a_motor3_position_guard.py"
        ).read_text(encoding="utf-8")
        for expected in (
            'PUBLIC_TOPIC = "/motor3_position_controller/commands_mm"',
            'RAW_TOPIC = "/motor3_raw_position_controller/commands"',
            'INTERFACE_NAME = "motor3_position"',
            "MIN_POSITION_MM = 0.0",
            "MAX_POSITION_MM = 20.0",
            "MIN_INCREMENT_MM = 0.0001",
            "MAX_VELOCITY_M_S = 0.008",
            "MAX_ACCELERATION_M_S2 = 0.050",
            'DynamicJointState, "/dynamic_joint_states"',
            "if not self.operator_command_received:",
        ):
            self.assertIn(expected, guard_text)

    def test_motor4_gui_guard_has_absolute_15mm_required_speed(self):
        guard_text = (
            MOTION_CMD / "motor_controller" / "azd3a_motor4_position_guard.py"
        ).read_text(encoding="utf-8")
        for expected in (
            'PUBLIC_TOPIC = "/motor4_position_controller/commands_mm"',
            'RAW_TOPIC = "/motor4_raw_position_controller/commands"',
            "MIN_POSITION_MM = 0.0",
            "MAX_POSITION_MM = 20.0",
            "MIN_INCREMENT_MM = 0.0001",
            "HARD_MAX_POSITION_MM = 30.0",
            "HARD_MAX_VELOCITY_M_S = 0.040",
            "HARD_MAX_ACCELERATION_M_S2 = 0.2",
            "REQUIRED_VELOCITY_M_S = 0.008",
            "DEFAULT_ACCELERATION_M_S2 = 0.050",
            "target_position = target_mm / 1000.0",
        ):
            self.assertIn(expected, guard_text)

    def test_motor6_conveyor_is_slave1_axis3_ps50_and_watchdog_guarded(self):
        slave_text = (
            MOTION_CMD / "config" / "azd3a_motor6_conveyor_slave.yaml"
        ).read_text(encoding="utf-8").lower()
        xacro_text = (
            MOTION_CMD / "urdf" / "azd3a_motor6_conveyor.urdf.xacro"
        ).read_text(encoding="utf-8")
        guard_text = (
            MOTION_CMD / "motor_controller" / "azd3a_motor6_conveyor_guard.py"
        ).read_text(encoding="utf-8")
        launch_text = (
            MOTION_CMD / "launch" / "azd3a_motor6_conveyor.launch.py"
        ).read_text(encoding="utf-8")

        self.assertRegex(slave_text, r"rpdo:\s*\n\s*-\s*index:\s*0x1622")
        self.assertRegex(slave_text, r"tpdo:\s*\n\s*-\s*index:\s*0x1a21")
        self.assertIn("factor: 79577.47154594767", slave_text)
        self.assertEqual(slave_text.count("factor: 0.000012566370614359173"), 2)
        self.assertIn('<param name="position">1</param>', xacro_text)
        self.assertIn('cia402_object_index_offset">0x1000', xacro_text)
        self.assertIn('<param name="number_of_physical_drives">2</param>', xacro_text)
        self.assertIn("azd3a_slave0_passive_keepalive.yaml", xacro_text)
        for expected in (
            "HARD_MAX_RPM = 60.0",
            "COMMISSIONING_MAX_RPM = 60.0",
            "DEFAULT_ACCELERATION_RPM_S = 25.0",
            "COMMAND_TIMEOUT_S = 0.5",
            'PUBLIC_TOPIC = "/motor6_conveyor/commands_rpm"',
            'RAW_TOPIC = "/motor6_raw_velocity_controller/commands"',
            "len(message.data) not in (1, 3)",
        ):
            self.assertIn(expected, guard_text)
        self.assertIn('executable="azd3a_motor6_conveyor_guard"', launch_text)

    def test_combined_slave1_maps_motor4_motor5_and_motor6(self):
        slave_text = (
            MOTION_CMD / "config" / "azd3a_motor4_motor6_slave.yaml"
        ).read_text(encoding="utf-8").lower()
        for pdo in ("0x1600", "0x1610", "0x1622", "0x1a00", "0x1a11", "0x1a21"):
            self.assertRegex(slave_text, rf"index:\s*{pdo}")
        self.assertIn("index: 0x607a", slave_text)
        self.assertIn("index: 0x70ff", slave_text)
        self.assertIn("index: 0x687a", slave_text)
        self.assertIn("index: 0x686c", slave_text)
        self.assertIn("command_interface: motor5_position", slave_text)
        self.assertIn("state_interface: motor6_position", slave_text)
        self.assertIn("factor: 79577.47154594767", slave_text)
        self.assertIn("factor: 0.000012566370614359173", slave_text)
        self.assertIn("factor: 31830.98861837907", slave_text)

    def test_motor5_combined_guard_requires_runtime_zero_and_signed_angle(self):
        guard_text = (
            MOTION_CMD / "motor_controller" / "azd3a_motor5_position_guard.py"
        ).read_text(encoding="utf-8")
        for expected in (
            'COMMAND_TOPIC = "/motor5_position_controller/commands_deg"',
            'SET_ZERO_TOPIC = "/motor5_position_controller/set_zero"',
            'MANUAL_STEP_TOPIC = "/motor5_position_controller/manual_step_deg"',
            'INTERFACE_NAME = "motor5_position"',
            "MAX_ANGLE_DEG = 180.0",
            "MAX_VELOCITY_RPM = 20.0",
            "MAX_ACCELERATION_RPM_S = 20.0",
            "self.origin = None",
            "self.origin = self.measured_position",
            "def on_manual_step",
            "set the runtime zero first",
            "self.origin + math.radians(offset_deg)",
        ):
            self.assertIn(expected, guard_text)

    def test_axis1_guard_encodes_hardware_caps_and_conservative_defaults(self):
        guard = MOTION_CMD / "motor_controller" / "azd3a_axis1_command_guard.py"
        launch = MOTION_CMD / "launch" / "azd3a_axis1_tiny_move.launch.py"
        xacro = MOTION_CMD / "urdf" / "azd3a_axis1_tiny_move.urdf.xacro"
        guard_text = guard.read_text(encoding="utf-8")
        launch_text = launch.read_text(encoding="utf-8")
        xacro_text = xacro.read_text(encoding="utf-8")

        for expected in (
            "HARD_MAX_POSITION_M = 0.400",
            "HARD_MAX_VELOCITY_M_S = 0.600",
            "HARD_MAX_ACCELERATION_M_S2 = 0.2",
            "MIN_TRAVEL_INCREMENT_M = 0.0000012",
            "DEFAULT_MIN_POSITION_M = 0.0",
            "DEFAULT_MAX_POSITION_M = 0.400",
            "DEFAULT_MAX_VELOCITY_M_S = 0.015",
            'PUBLIC_TOPIC = "/axis1_position_controller/commands"',
            'RAW_TOPIC = "/axis1_raw_position_controller/commands"',
            "self.operator_command_received = False",
            "if not self.operator_command_received:",
            "self.operator_command_received = True",
        ):
            self.assertIn(expected, guard_text)
        self.assertIn('"min_position_m": 0.0', launch_text)
        self.assertIn('"max_position_m": 0.400', launch_text)
        self.assertIn('"max_velocity_m_s": 0.015', launch_text)
        self.assertIn('"max_acceleration_m_s2": 0.015', launch_text)
        self.assertIn("position_startup_tolerance\">0.00001", xacro_text)

    def test_motor4_commissioning_is_slave1_relative_and_tightly_bounded(self):
        slave_text = (
            MOTION_CMD / "config" / "azd3a_motor4_commissioning_slave.yaml"
        ).read_text(encoding="utf-8").lower()
        keepalive_text = (
            MOTION_CMD / "config" / "azd3a_slave0_passive_keepalive.yaml"
        ).read_text(encoding="utf-8").lower()
        xacro_text = (
            MOTION_CMD / "urdf" / "azd3a_motor4_commissioning.urdf.xacro"
        ).read_text(encoding="utf-8")
        guard_text = (
            MOTION_CMD
            / "motor_controller"
            / "azd3a_motor4_commissioning_guard.py"
        ).read_text(encoding="utf-8")

        self.assertRegex(slave_text, r"rpdo:\s*\n\s*-\s*index:\s*0x1600")
        self.assertRegex(slave_text, r"tpdo:\s*\n\s*-\s*index:\s*0x1a00")
        self.assertIn("factor: 10000000.0", slave_text)
        self.assertIn("factor: 0.0000001", slave_text)
        self.assertIn("default: .nan", slave_text)
        self.assertIn("auto_fault_reset: false", slave_text)
        self.assertIn('<param name="position">1</param>', xacro_text)
        self.assertIn('<param name="number_of_physical_drives">2</param>', xacro_text)
        self.assertIn("GenericEcSlave", xacro_text)
        self.assertIn('<param name="position">0</param>', xacro_text)
        self.assertIn("azd3a_slave0_passive_keepalive.yaml", xacro_text)
        self.assertIn('<param name="mode_of_operation">8</param>', xacro_text)
        self.assertIn("MAX_OFFSET_MM = 0.500", guard_text)
        self.assertIn("MIN_INCREMENT_MM = 0.001", guard_text)
        self.assertIn("MAX_VELOCITY_M_S = 0.002", guard_text)
        self.assertIn('PUBLIC_TOPIC = "/motor4_commissioning/offset_mm"', guard_text)
        self.assertIn("-MAX_OFFSET_MM <= offset_mm <= MAX_OFFSET_MM", guard_text)
        self.assertIn("self.origin = position", guard_text)
        self.assertIn("self.origin + offset_mm / 1000.0", guard_text)
        self.assertIn("command_interface: ~, default: 0", keepalive_text)
        self.assertNotIn("command_interface: position", keepalive_text)

    def test_motor3_commissioning_is_slave0_axis3_relative_and_bounded(self):
        slave_text = (
            MOTION_CMD / "config" / "azd3a_motor3_commissioning_slave.yaml"
        ).read_text(encoding="utf-8").lower()
        xacro_text = (
            MOTION_CMD / "urdf" / "azd3a_motor3_commissioning.urdf.xacro"
        ).read_text(encoding="utf-8")
        guard_text = (
            MOTION_CMD / "motor_controller" / "azd3a_motor3_commissioning_guard.py"
        ).read_text(encoding="utf-8")
        launch_text = (
            MOTION_CMD / "launch" / "azd3a_motor3_commissioning.launch.py"
        ).read_text(encoding="utf-8")
        self.assertRegex(slave_text, r"rpdo:\s*\n\s*-\s*index:\s*0x1620")
        self.assertRegex(slave_text, r"tpdo:\s*\n\s*-\s*index:\s*0x1a20")
        self.assertIn("factor: 10000000.0", slave_text)
        self.assertIn("factor: 0.0000001", slave_text)
        self.assertIn('cia402_object_index_offset">0x1000', xacro_text)
        self.assertIn('<param name="position">0</param>', xacro_text)
        self.assertIn("azd3a_slave1_passive_keepalive.yaml", xacro_text)
        self.assertIn("MAX_OFFSET_MM = 2.000", guard_text)
        self.assertIn("MAX_VELOCITY_M_S = 0.008", guard_text)
        self.assertIn("MAX_ACCELERATION_M_S2 = 0.050", guard_text)
        self.assertIn('PUBLIC_TOPIC = "/motor3_commissioning/offset_mm"', guard_text)
        self.assertIn('executable="azd3a_motor3_commissioning_guard"', launch_text)

    def test_motor5_commissioning_is_slave1_axis2_relative_and_tightly_bounded(self):
        slave_text = (
            MOTION_CMD / "config" / "azd3a_motor5_commissioning_slave.yaml"
        ).read_text(encoding="utf-8").lower()
        xacro_text = (
            MOTION_CMD / "urdf" / "azd3a_motor5_commissioning.urdf.xacro"
        ).read_text(encoding="utf-8")
        guard_text = (
            MOTION_CMD / "motor_controller" / "azd3a_motor5_commissioning_guard.py"
        ).read_text(encoding="utf-8")
        launch_text = (
            MOTION_CMD / "launch" / "azd3a_motor5_commissioning.launch.py"
        ).read_text(encoding="utf-8")
        self.assertRegex(slave_text, r"rpdo:\s*\n\s*-\s*index:\s*0x1610")
        self.assertRegex(slave_text, r"tpdo:\s*\n\s*-\s*index:\s*0x1a11")
        self.assertIn("factor: 31830.98861837907", slave_text)
        self.assertIn("factor: 0.00003141592653589793", slave_text)
        self.assertIn('cia402_object_index_offset">0x0800', xacro_text)
        self.assertIn('<param name="position">1</param>', xacro_text)
        self.assertIn("azd3a_slave0_passive_keepalive.yaml", xacro_text)
        self.assertIn("MAX_OFFSET_DEG = 10.0", guard_text)
        self.assertIn("MAX_VELOCITY_RPM = 5.0", guard_text)
        self.assertIn("MAX_ACCELERATION_RPM_S = 5.0", guard_text)
        self.assertIn('PUBLIC_TOPIC = "/motor5_commissioning/offset_deg"', guard_text)
        self.assertIn("self.origin + math.radians(offset_deg)", guard_text)
        self.assertIn("-MAX_OFFSET_DEG <= offset_deg <= MAX_OFFSET_DEG", guard_text)
        self.assertIn('executable="azd3a_motor5_commissioning_guard"', launch_text)


if __name__ == "__main__":
    unittest.main()
