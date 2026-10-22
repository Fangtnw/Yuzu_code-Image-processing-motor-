from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "motion_cmd"))
from motor_controller.yuzu_mock_contour import yellow_points
from motor_controller.yuzu_contour_profile import ContourProfile


class MockTests(unittest.TestCase):
    def test_circle_has_known_horizontal_diameter(self):
        profile = ContourProfile(yellow_points(30, 30, 3))
        self.assertEqual(profile.horizontal_diameter_mm(), 60)

    def test_vertical_radius_does_not_change_horizontal_diameter(self):
        data = yellow_points()
        self.assertEqual(len(data), 33)
        self.assertEqual(data[::3], list(range(30, 360, 30)))
        profile = ContourProfile(data)
        self.assertEqual(profile.horizontal_diameter_mm(), 60)
        self.assertEqual(ContourProfile(yellow_points(30, 40, 3)).horizontal_diameter_mm(), 60)

    def test_bad_dimensions(self):
        for dims in ((0, 27, 3), (30, 27, 27), (30, 27, -1), (float("nan"), 27, 3)):
            with self.assertRaises(ValueError):
                yellow_points(*dims)
