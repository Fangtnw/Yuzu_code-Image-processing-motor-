import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "motion_cmd"))
from motor_controller.yuzu_motor5_orbit_plan import orbit_waypoints  # noqa: E402


class Motor5OrbitPlanTests(unittest.TestCase):
    def test_default_sequence_crosses_zero_and_returns(self):
        plan = orbit_waypoints()
        self.assertEqual([(item.phase, item.motor5_target_deg) for item in plan[:3]], [
            ("side_zero", 0), ("standby_cw", 90), ("slides_feed_in", 90),
        ])
        sweep = [item.motor5_target_deg for item in plan if item.phase == "peel_sweep_ccw"]
        self.assertEqual(sweep, list(range(85, -11, -5)))
        self.assertTrue(all(item.update_slides for item in plan[2:-2]))
        self.assertEqual((plan[-2].phase, plan[-2].motor5_target_deg,
                          plan[-2].update_slides), ("slides_retract", -10, False))
        self.assertEqual((plan[-1].phase, plan[-1].motor5_target_deg),
                         ("return_to_origin", 0))

    def test_physical_clockwise_can_be_negative(self):
        plan = orbit_waypoints(clockwise_sign=-1)
        self.assertEqual(plan[1].motor5_target_deg, -90)
        self.assertEqual(plan[-2].motor5_target_deg, 10)

    def test_rejects_unreachable_or_nonuniform_sweep(self):
        for kwargs in ({"clockwise_sign": 0}, {"step_deg": 7},
                       {"overshoot_deg": 11}, {"overshoot_deg": 100}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                orbit_waypoints(**kwargs)


if __name__ == "__main__":
    unittest.main()
