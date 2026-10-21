from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "motion_cmd"))
from motor_controller.yuzu_mock_contour import yellow_points
from motor_controller.yuzu_contour_profile import ContourProfile


class MockTests(unittest.TestCase):
    def test_circle_known_feed(self):
        profile = ContourProfile(yellow_points(30, 30, 3))
        for angle in (-45, -30, 0, 30, 45):
            self.assertEqual(profile.feed_at(angle, 90, 1), 5)

    def test_ellipse_moves_feed(self):
        data = yellow_points()
        self.assertEqual(len(data), 33)
        self.assertEqual(data[::3], list(range(30, 360, 30)))
        profile = ContourProfile(data)
        center = profile.feed_at(0, 90, 1)
        self.assertGreater(profile.feed_at(30, 90, 1), center)
        self.assertEqual(profile.feed_at(-30, 90, 1), profile.feed_at(30, 90, 1))

    def test_bad_dimensions(self):
        for dims in ((0, 27, 3), (30, 27, 27), (30, 27, -1), (float("nan"), 27, 3)):
            with self.assertRaises(ValueError):
                yellow_points(*dims)
