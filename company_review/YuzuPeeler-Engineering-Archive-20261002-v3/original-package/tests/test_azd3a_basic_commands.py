import sys
import unittest
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "motion_cmd"
sys.path.insert(0, str(PACKAGE_ROOT))

from motor_controller.azd3a_basic_commands import (  # noqa: E402
    AxisLimits,
    build_angle_command,
    build_move_mm_command,
    build_spin_rpm_command,
)


class Azd3aBasicCommandTests(unittest.TestCase):
    def test_build_move_mm_command_uses_position_speed_accel_and_decel_payload(self):
        command = build_move_mm_command(
            position_mm=2.5,
            speed_mm_s=1.2,
            accel_mm_s2=4.0,
            decel_mm_s2=3.0,
            limits=AxisLimits(min_position_mm=-10.0, max_position_mm=10.0),
        )

        self.assertEqual(command.topic, "motor_cmd")
        self.assertEqual(command.message_type, "Float32MultiArray")
        self.assertEqual(command.values, [2.5, 1.2, 4.0, 3.0])

    def test_build_move_mm_command_rejects_position_outside_soft_limits(self):
        with self.assertRaisesRegex(ValueError, "position_mm 20.0 outside"):
            build_move_mm_command(
                position_mm=20.0,
                speed_mm_s=1.0,
                limits=AxisLimits(min_position_mm=-5.0, max_position_mm=5.0),
            )

    def test_build_spin_rpm_command_uses_rpm_accel_and_decel_payload(self):
        command = build_spin_rpm_command(rpm=30.0, accel_rpm_s=10.0, decel_rpm_s=8.0)

        self.assertEqual(command.topic, "motor_spin")
        self.assertEqual(command.message_type, "Float32MultiArray")
        self.assertEqual(command.values, [30.0, 10.0, 8.0])

    def test_build_angle_command_uses_angle_speed_rpm_accel_and_decel_payload(self):
        command = build_angle_command(
            angle_deg=45.0,
            speed_rpm=12.0,
            accel_rpm_s=6.0,
            decel_rpm_s=5.0,
        )

        self.assertEqual(command.topic, "motor_angle_cmd")
        self.assertEqual(command.message_type, "Float32MultiArray")
        self.assertEqual(command.values, [45.0, 12.0, 6.0, 5.0])

    def test_build_angle_command_rejects_angle_outside_soft_limits(self):
        with self.assertRaisesRegex(ValueError, "angle_deg 370.0 outside"):
            build_angle_command(
                angle_deg=370.0,
                speed_rpm=5.0,
                limits=AxisLimits(min_angle_deg=-180.0, max_angle_deg=180.0),
            )


if __name__ == "__main__":
    unittest.main()
