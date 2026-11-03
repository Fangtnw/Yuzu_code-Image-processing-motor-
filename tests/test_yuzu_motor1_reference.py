from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "motion_cmd"))
from motor_controller.yuzu_motor1_reference import Motor1CameraReference


class Motor1ReferenceTests(unittest.TestCase):
    def test_camera_offset_target_accounts_for_detector_inset(self):
        targets = Motor1CameraReference().targets(20, 120)
        self.assertAlmostEqual(targets["insertion_target_mm"], 184.0, delta=0.0006)
        self.assertAlmostEqual(targets["lift_target_mm"], 64.0, delta=0.0006)
        self.assertEqual(targets["y_offset_mm"], 20)

    def test_camera_offset_is_signed_and_depth_is_relative_to_outer_top(self):
        target = Motor1CameraReference(pierce_depth_mm=5).targets(-10)[
            "insertion_target_mm"]
        self.assertAlmostEqual(target, 216.0, delta=0.0006)
        extra = Motor1CameraReference(allowance_mm=2).targets(-10)[
            "insertion_target_mm"]
        self.assertAlmostEqual(extra, 216.0, delta=0.0006)

    def test_camera_target_rejects_invalid_calibration_and_stroke(self):
        for kwargs, offset in (({"frame_center_position_mm": 0}, 10),
                               ({}, float("nan")),
                               ({"frame_center_position_mm": 390}, -20),
                               ({"offset_multiplier": 0}, 10)):
            with self.subTest(kwargs=kwargs, offset=offset), self.assertRaises(ValueError):
                Motor1CameraReference(**kwargs).targets(offset)

    def test_rejects_invalid_lift(self):
        with self.assertRaises(ValueError):
            Motor1CameraReference().targets(10, 151)


if __name__ == "__main__":
    unittest.main()
