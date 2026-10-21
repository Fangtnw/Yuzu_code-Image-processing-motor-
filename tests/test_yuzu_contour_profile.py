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
    def test_inset_applied_once(self):
        p = ContourProfile(circle(27))  # surface radius 30, detector inset 3
        self.assertEqual(p.feed_at(0, 90, 1, peel_depth_mm=1), 6)
        self.assertEqual(p.feed_at(45, 90, 1), 5)

    def test_interpolation_and_direction(self):
        data = circle(25)
        data[6:9] = [90, 30, 0]
        p = ContourProfile(data)
        self.assertAlmostEqual(p.radius_at(75), 27.5)
        self.assertEqual(p.feed_at(30, 90, -1), 7)

    def test_missing_red_point(self):
        p = ContourProfile(circle(27))
        for angle in (0, 15, 345, 360):
            with self.assertRaises(ValueError):
                p.radius_at(angle)

    def test_unreachable_screenshot_scale(self):
        with self.assertRaisesRegex(ValueError, "Unreachable"):
            ContourProfile(circle(50)).feed_at(0, 90, 1)

    def test_invalid_messages(self):
        for data in ([], circle(25)[:-1], [float("nan")] * 33, circle(0)):
            with self.assertRaises(ValueError):
                ContourProfile(data)
        data = circle(25)
        data[3] = 30
        with self.assertRaises(ValueError):
            ContourProfile(data)


if __name__ == "__main__":
    unittest.main()
