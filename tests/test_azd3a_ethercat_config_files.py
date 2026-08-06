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
        self.assertIn("factor: 10000000.0", text)
        self.assertIn("factor: 0.0000001", text)
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
            'lower="0.000" upper="0.030" velocity="0.040"', xacro_text
        )
        self.assertIn('<param name="max">0.005</param>', xacro_text)
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
        self.assertEqual(slave_text.count("factor: 0.0006283185307179586"), 2)
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
        slave_text = slave.read_text(encoding="utf-8").lower()
        xacro_text = xacro.read_text(encoding="utf-8")
        guard_text = guard.read_text(encoding="utf-8")

        self.assertRegex(slave_text, r"rpdo:\s*\n\s*-\s*index:\s*0x1612")
        self.assertRegex(slave_text, r"tpdo:\s*\n\s*-\s*index:\s*0x1a11")
        self.assertIn("command_interface: velocity", slave_text)
        self.assertIn("factor: 1591.5494309189535", slave_text)
        self.assertIn("default: 0", slave_text)
        self.assertIn("mode_of_operation\">9", xacro_text)
        self.assertIn("cia402_object_index_offset\">0x0800", xacro_text)
        self.assertIn("2.6179938779914944", xacro_text)
        for expected in (
            "HARD_MAX_RPM = 416.0",
            "COMMISSIONING_MAX_RPM = 25.0",
            "DEFAULT_ACCELERATION_RPM_S = 10.0",
            "COMMAND_TIMEOUT_S = 0.5",
            'PUBLIC_TOPIC = "/axis2_velocity_controller/commands_rpm"',
            'RAW_TOPIC = "/axis2_raw_velocity_controller/commands"',
        ):
            self.assertIn(expected, guard_text)

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
        self.assertNotIn("scripts/*.sh", text)

    def test_axis1_guard_encodes_hardware_caps_and_conservative_defaults(self):
        guard = MOTION_CMD / "motor_controller" / "azd3a_axis1_command_guard.py"
        launch = MOTION_CMD / "launch" / "azd3a_axis1_tiny_move.launch.py"
        xacro = MOTION_CMD / "urdf" / "azd3a_axis1_tiny_move.urdf.xacro"
        guard_text = guard.read_text(encoding="utf-8")
        launch_text = launch.read_text(encoding="utf-8")
        xacro_text = xacro.read_text(encoding="utf-8")

        for expected in (
            "HARD_MAX_POSITION_M = 0.030",
            "HARD_MAX_VELOCITY_M_S = 0.040",
            "HARD_MAX_ACCELERATION_M_S2 = 0.2",
            "MIN_TRAVEL_INCREMENT_M = 0.000001",
            'PUBLIC_TOPIC = "/axis1_position_controller/commands"',
            'RAW_TOPIC = "/axis1_raw_position_controller/commands"',
            "self.operator_command_received = False",
            "if not self.operator_command_received:",
            "self.operator_command_received = True",
        ):
            self.assertIn(expected, guard_text)
        self.assertIn('"max_position_m": 0.015', launch_text)
        self.assertIn('"max_velocity_m_s": 0.0005', launch_text)
        self.assertIn('"max_acceleration_m_s2": 0.001', launch_text)
        self.assertIn("position_startup_tolerance\">0.00001", xacro_text)


if __name__ == "__main__":
    unittest.main()
