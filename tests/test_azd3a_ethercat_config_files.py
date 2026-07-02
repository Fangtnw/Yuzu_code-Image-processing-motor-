import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
MOTION_CMD = REPO_ROOT / "motion_cmd"


class Azd3aEthercatConfigFileTests(unittest.TestCase):
    def test_slave_yaml_matches_azd_ked_identity_and_required_cia402_pdos(self):
        config = MOTION_CMD / "config" / "azd3a_ked_cia402_slave.yaml"
        text = config.read_text(encoding="utf-8")

        self.assertIn("vendor_id: 0x000002BE", text)
        self.assertIn("product_id: 0x000013E5", text)
        for index in ("0x6040", "0x607a", "0x60ff", "0x6060"):
            self.assertRegex(text.lower(), rf"index:\s*{index}")
        for index in ("0x6041", "0x6064", "0x6061"):
            self.assertRegex(text.lower(), rf"index:\s*{index}")
        self.assertIn("auto_fault_reset: false", text)

    def test_workspace_does_not_carry_ros2_control_robot_description_artifacts(self):
        self.assertFalse((MOTION_CMD / "urdf" / "azd3a_one_axis.ros2_control.xacro").exists())
        self.assertFalse((MOTION_CMD / "launch" / "azd3a_ethercat_driver.launch.py").exists())

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
