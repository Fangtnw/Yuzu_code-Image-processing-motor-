from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "motion_cmd"))
from motor_controller.motor5_gui_steps import stepped_motor5_target


class Motor5GuiStepTests(unittest.TestCase):
    def test_left_button_crosses_zero_and_reaches_negative_limit(self):
        self.assertEqual(stepped_motor5_target(0, 1, 90, -1), -90)
        self.assertEqual(stepped_motor5_target(90, -1, 90, -1), -180)
        self.assertEqual(stepped_motor5_target(180, -1, 90, -1), -180)

    def test_right_button_crosses_zero_and_reaches_positive_limit(self):
        self.assertEqual(stepped_motor5_target(90, -1, 90, 1), 0)
        self.assertEqual(stepped_motor5_target(0, -1, 90, 1), 90)
        self.assertEqual(stepped_motor5_target(180, 1, 90, 1), 180)

    def test_invalid_values_are_rejected(self):
        for args in ((0, 1, 0, -1), (0, 1, float("nan"), -1),
                     (181, 1, 1, -1), (0, 0, 1, -1)):
            with self.assertRaises(ValueError):
                stepped_motor5_target(*args)


if __name__ == "__main__":
    unittest.main()
