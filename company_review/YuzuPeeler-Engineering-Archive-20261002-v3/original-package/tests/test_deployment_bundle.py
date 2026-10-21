"""Offline checks for the portable deployment contract."""
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class DeploymentTests(unittest.TestCase):
    def test_readme_archive_installation_contract(self):
        readme = (ROOT / "README.md").read_text()
        self.assertNotRegex(readme.lower(), r"clone\s+(this|the)\s+(repo|repository)")
        for instruction in (
            "Extract the supplied ZIP",
            "Keep this folder in place",
            "Internet access is required",
            "bash scripts/setup_workspace.sh",
            "bash scripts/start_yuzu_peeler.sh",
            "sudo /etc/init.d/ethercat start",
            "--packages-up-to motor_controller",
        ):
            self.assertIn(instruction, readme)
        for relative in ("scripts/setup_workspace.sh", "scripts/start_yuzu_peeler.sh",
                         "UBUNTU_ETHERCAT_SETUP_GUIDE.md",
                         "motion_cmd/YUZU_OPERATOR_GUI.md",
                         "motion_cmd/launch/yuzu_peeler.launch.py", "patches/README.md"):
            self.assertTrue((ROOT / relative).is_file(), relative)

    def test_shell_syntax_and_permissions(self):
        for name in ("setup_workspace.sh", "start_yuzu_peeler.sh", "prepare_driver.sh"):
            path = ROOT / "scripts" / name
            subprocess.run(["bash", "-n", str(path)], check=True)
            self.assertTrue(path.stat().st_mode & 0o111, name)

    def test_dependency_closure(self):
        package = ET.parse(ROOT / "motion_cmd/package.xml")
        dependencies = {item.text for item in package.findall("exec_depend")}
        self.assertTrue({"ethercat_driver", "ethercat_generic_cia402_drive",
                         "velocity_controllers"} <= dependencies)
        setup = (ROOT / "scripts/setup_workspace.sh").read_text()
        self.assertIn("--packages-up-to motor_controller", setup)
        self.assertLess(setup.index('bash "$REPO_ROOT/scripts/prepare_driver.sh"'),
                        setup.index("colcon build"))

    def test_patch_contains_implementation_and_tests(self):
        patch = (ROOT / "patches/ethercat-driver-multiaxis.patch").read_text()
        self.assertIn("tertiary_axis_enabled_", patch)
        self.assertIn("test_generic_ec_cia402_drive.cpp", patch)

    def test_ros_sourcing_is_nounset_safe(self):
        # ROS setup availability is optional for a source-only reviewer.
        if not Path("/opt/ros/humble/setup.bash").exists():
            self.skipTest("ROS Humble is not installed")
        for name in ("setup_workspace.sh", "start_yuzu_peeler.sh"):
            script = (ROOT / "scripts" / name).read_text()
            snippet = "set +u\nsource /opt/ros/humble/setup.bash\nset -u"
            self.assertIn(snippet, script)
            subprocess.run(["bash", "--noprofile", "--norc", "-uc", snippet],
                           env={"PATH": "/usr/bin:/bin"}, check=True)
