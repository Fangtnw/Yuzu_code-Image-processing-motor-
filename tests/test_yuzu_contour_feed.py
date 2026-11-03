import math
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "motion_cmd"))
from motor_controller.yuzu_contour_feed import ContourFeedProfile  # noqa: E402
from motor_controller.yuzu_contour_feed_preview import build_rows  # noqa: E402
from motor_controller.yuzu_step3_plan import build_step3_actions, interpolated_feed  # noqa: E402


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
        self.assertEqual(targets["motor3_target_mm"], 15)
        self.assertEqual(targets["motor4_target_mm"], 15)

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
        self.assertEqual(targets["motor3_target_mm"], 15)
        self.assertEqual(targets["motor4_target_mm"], 16)

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
        self.assertTrue(all(row["motor3_target_mm"] == 15 and
                            row["motor4_target_mm"] == 15 for row in rows))

    def test_out_of_stroke_error_identifies_blade_and_sweep_position(self):
        data = profile_data()
        i = (180 // 5) * 3
        data[i + 1:i + 3] = [0, -11]
        with self.assertRaisesRegex(ValueError, "Motor 4 at Motor 5 \\+90 degrees.*21.000"):
            build_rows(data)

    def test_outer_surface_correction_resolves_live_sized_bottom_radius(self):
        data = profile_data()
        i = (180 // 5) * 3
        data[i + 1:i + 3] = [0, -13.9]
        rows = build_rows(data)
        self.assertAlmostEqual(rows[0]["motor4_target_mm"], 18.1, places=3)
        with self.assertRaisesRegex(ValueError, "Motor 4 at Motor 5 \\+90"):
            build_rows(data, source_inset_mm=0)

    def test_configured_peel_depth_is_added_once(self):
        rows = build_rows(profile_data(), peel_depth_mm=1)
        self.assertEqual(rows[0]["motor3_target_mm"], 16)
        with self.assertRaises(ValueError):
            build_rows(profile_data(), source_inset_mm=-1)

    def test_step3_plan_retracts_before_returning_orbit_to_zero(self):
        actions = build_step3_actions(profile_data())
        self.assertEqual(len(actions), 24)
        self.assertEqual((actions[0].phase, actions[0].command_motors), ("standby", (5,)))
        self.assertEqual((actions[1].phase, actions[1].command_motors), ("feed_start", (3, 4)))
        self.assertEqual(actions[2].command_motors, (3, 4, 5))
        self.assertEqual((actions[-2].phase, actions[-2].motor5_deg,
                          actions[-2].motor3_mm, actions[-2].motor4_mm),
                         ("release", -10, 0, 0))
        self.assertEqual((actions[-1].phase, actions[-1].motor5_deg,
                          actions[-1].command_motors), ("return_to_origin", 0, (5,)))

    def test_continuous_feed_interpolates_between_frozen_five_degree_samples(self):
        data = profile_data()
        angle = 5
        i = (angle // 5) * 3
        data[i + 1:i + 3] = [18 * math.sin(math.radians(angle)),
                             18 * math.cos(math.radians(angle))]
        actions = build_step3_actions(data)
        self.assertEqual(interpolated_feed(actions, 90), (15, 15))
        self.assertEqual(interpolated_feed(actions, 87.5),
                         (round((actions[1].motor3_mm + actions[2].motor3_mm) / 2, 4),
                          round((actions[1].motor4_mm + actions[2].motor4_mm) / 2, 4)))
        self.assertEqual(interpolated_feed(actions, -10), (15, 15))
        with self.assertRaises(ValueError):
            interpolated_feed(actions, -10.1)


if __name__ == "__main__":
    unittest.main()
