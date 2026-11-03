import math
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "motion_cmd"))
from motor_controller.yuzu_contour_feed import ContourFeedProfile  # noqa: E402
from motor_controller.yuzu_contour_feed_preview import build_rows  # noqa: E402


def profile_data(radius=17.0):
    return [value for angle in range(0, 360, 5)
            for value in (angle,
                          radius * math.sin(math.radians(angle)),
                          radius * math.cos(math.radians(angle)))]


class ContourFeedTests(unittest.TestCase):
    def test_red_zero_and_bottom_180_feed_from_35_mm(self):
        profile = ContourFeedProfile(profile_data())
        targets = profile.targets_at_motor5(90, camera_angle_direction=1)
        self.assertEqual(targets["motor3_contour_angle_deg"], 0)
        self.assertEqual(targets["motor4_contour_angle_deg"], 180)
        self.assertEqual(targets["motor3_target_mm"], 18)
        self.assertEqual(targets["motor4_target_mm"], 18)

    def test_5_degree_measured_radius_and_periodic_wrap(self):
        data = profile_data()
        data[1:3] = [0, 16]
        data[4:6] = [18 * math.sin(math.radians(5)),
                     18 * math.cos(math.radians(5))]
        profile = ContourFeedProfile(data)
        self.assertAlmostEqual(profile.radius_mm(5), 18)
        self.assertAlmostEqual(profile.radius_mm(360), 16)
        with self.assertRaises(ValueError):
            profile.radius_mm(2.5)

    def test_angle_direction_is_explicit(self):
        profile = ContourFeedProfile(profile_data())
        self.assertEqual(profile.targets_at_motor5(-10, 1)["motor3_contour_angle_deg"], 100)
        self.assertEqual(profile.targets_at_motor5(-10, -1)["motor3_contour_angle_deg"], 260)
        self.assertEqual(profile.targets_at_motor5(-10, 1)["motor4_contour_angle_deg"], 280)
        self.assertEqual(profile.targets_at_motor5(-10, -1)["motor4_contour_angle_deg"], 80)

    def test_mirror_radii_are_averaged_per_blade(self):
        data = profile_data()
        for angle, radius in ((30, 16), (330, 18), (210, 15), (150, 17)):
            i = (angle // 5) * 3
            data[i + 1:i + 3] = [radius * math.sin(math.radians(angle)),
                                  radius * math.cos(math.radians(angle))]
        targets = ContourFeedProfile(data).targets_at_motor5(60)
        self.assertEqual((targets["motor3_contour_angle_deg"],
                          targets["motor3_mirror_angle_deg"]), (30, 330))
        self.assertEqual((targets["motor4_contour_angle_deg"],
                          targets["motor4_mirror_angle_deg"]), (210, 150))
        self.assertEqual(targets["motor3_target_mm"], 18)
        self.assertEqual(targets["motor4_target_mm"], 19)

    def test_missing_red_point_and_out_of_stroke_are_rejected(self):
        with self.assertRaises(ValueError):
            ContourFeedProfile(profile_data()[3:])
        with self.assertRaises(ValueError):
            ContourFeedProfile(profile_data(radius=10)).slide_target_mm(0)
        with self.assertRaises(ValueError):
            ContourFeedProfile(profile_data()).targets_at_motor5(90, 0)

    def test_preview_covers_standby_through_overshoot_without_retract_motion(self):
        rows = build_rows(profile_data())
        self.assertEqual(len(rows), 21)
        self.assertEqual(rows[0]["motor5_deg"], 90)
        self.assertEqual(rows[-1]["motor5_deg"], -10)
        self.assertTrue(all(row["motor3_target_mm"] == 18 and
                            row["motor4_target_mm"] == 18 for row in rows))


if __name__ == "__main__":
    unittest.main()
