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
        self.assertIn("auto_fault_reset: false", text)

    def test_feedback_launch_cannot_auto_enable_cia402_or_load_motion_controller(self):
        xacro = MOTION_CMD / "urdf" / "azd3a_axis1_feedback.urdf.xacro"
        launch = MOTION_CMD / "launch" / "azd3a_axis1_feedback.launch.py"
        xacro_text = xacro.read_text(encoding="utf-8")
        launch_text = launch.read_text(encoding="utf-8")

        self.assertIn("ethercat_generic_plugins/GenericEcSlave", xacro_text)
        self.assertNotIn("ethercat_generic_plugins/EcCiA402Drive", xacro_text)
        self.assertIn('name="axis1_joint" type="prismatic"', xacro_text)
        self.assertIn('lower="0.000" upper="0.029"', xacro_text)
        self.assertIn("joint_state_broadcaster", launch_text)
        self.assertIn("ParameterValue", launch_text)
        self.assertIn("value_type=str", launch_text)
        self.assertNotIn("trajectory", launch_text)
        self.assertNotIn("forward_command", launch_text)

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


if __name__ == "__main__":
    unittest.main()
