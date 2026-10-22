import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "motion_cmd"))
from motor_controller.yuzu_contour_profile import ContourProfile


def circle(radius):
    return [v for a in range(30, 360, 30) for v in
            (a, radius * math.sin(math.radians(a)), radius * math.cos(math.radians(a)))]


class ProfileTests(unittest.TestCase):
    def test_horizontal_diameter_restores_detector_inset(self):
        profile = ContourProfile(circle(27))
        self.assertEqual(profile.horizontal_diameter_mm(), 60)
        self.assertEqual(profile.horizontal_diameter_mm(0), 54)

    def test_left_and_right_radii_both_contribute(self):
        data = circle(25)
        data[6:9] = [90, 30, 0]
        profile = ContourProfile(data)
        self.assertEqual(profile.horizontal_diameter_mm(), 61)

    def test_invalid_messages(self):
        for data in ([], circle(25)[:-1], [float("nan")] * 33, circle(0)):
            with self.assertRaises(ValueError):
                ContourProfile(data)
        data = circle(25)
        data[3] = 30
        with self.assertRaises(ValueError):
            ContourProfile(data)
        data = circle(25)
        data[7] = -25
        with self.assertRaises(ValueError):
            ContourProfile(data)

    def test_invalid_source_inset(self):
        profile = ContourProfile(circle(25))
        for inset in (-1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                profile.horizontal_diameter_mm(inset)


if __name__ == "__main__":
    unittest.main()
