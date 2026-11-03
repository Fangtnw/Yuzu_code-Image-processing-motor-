"""CSP trajectory regression checks without driving hardware."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'motion_cmd'))
from motor_controller.conveyor_index import ConveyorIndex, MM_PER_COUNT, OPERATING_MAX_RPM, TRANSMISSION_RATIO, TRAVEL_CALIBRATION


class CspTests(unittest.TestCase):
    def test_confirmed_ratio_preserves_physical_calibration(self):
        self.assertAlmostEqual(TRANSMISSION_RATIO, 25 / 18)
        self.assertAlmostEqual(TRAVEL_CALIBRATION, 1.0224)
        self.assertAlmostEqual(1 / MM_PER_COUNT, 3736.031527978765)
        self.assertAlmostEqual(OPERATING_MAX_RPM, 60.0)

    def test_velocity_noise_with_stationary_encoder(self):
        motion = ConveyorIndex()
        for i in range(101):
            motion.feedback(1000000 + i % 2, .05 if i % 3 == 0 else 0, True, i*.005)
        self.assertTrue(motion.stopped(.5))

    def test_slow_encoder_drift_never_reports_stopped(self):
        motion = ConveyorIndex()
        for i in range(101):
            motion.feedback(1000000 + i, 0, True, i*.005)
            self.assertFalse(motion.stopped(i*.005))

    def run_motion(self, jog=False, stalled=False):
        motion = ConveyorIndex()
        origin = 2050367
        for i in range(41):
            motion.feedback(origin, 0, True, i * .005)
        motion.capture_reference(.2)
        if jog:
            motion.jog(35.995, OPERATING_MAX_RPM, 120, 120, .2)
        else:
            motion.request_index(1, OPERATING_MAX_RPM, 120, 120, .2)
        for i in range(1, 1601):
            now = .2 + i * .005
            motion.heartbeat(now)
            position = origin if stalled else round(motion.command_counts)
            motion.feedback(position, 0 if stalled else motion.output_rpm, True, now)
            motion.update(now)
        return motion, origin

    def test_nonzero_origin_index(self):
        motion, origin = self.run_motion()
        self.assertEqual(motion.state, 'ready')
        self.assertAlmostEqual((motion.position_counts-origin)*MM_PER_COUNT, 140.0, delta=.02)

    def test_jog_moves_and_clears_reference(self):
        motion, origin = self.run_motion(jog=True)
        self.assertEqual(motion.state, 'unreferenced')
        self.assertIsNone(motion.origin_counts)
        self.assertAlmostEqual((motion.position_counts-origin)*MM_PER_COUNT, 35.995, delta=.02)

    def test_stalled_encoder_cannot_complete(self):
        motion, _ = self.run_motion(stalled=True)
        self.assertEqual(motion.state, 'interrupted')
        self.assertIn('timeout', motion.message)

    def test_discontinuity_stays_blocked(self):
        motion = ConveyorIndex()
        motion.feedback(0, 0, True, 0)
        motion.feedback(2754581, 0, True, .005)
        self.assertFalse(motion.feedback_fault)
        motion.feedback(2754581 * 2, 0, True, .010)
        self.assertTrue(motion.feedback_fault)
        diagnostic = motion.status(.010)["last_discontinuity"]
        self.assertEqual(diagnostic["delta_counts"], 2754581)
        self.assertAlmostEqual(diagnostic["elapsed_s"], .005)
        self.assertGreater(abs(diagnostic["delta_mm"]), diagnostic["allowed_mm"])
        self.assertIn("allowed", motion.message)
        motion.feedback(0, 0, True, .010)
        self.assertFalse(motion.fresh(.010))
        self.assertFalse(motion.enabled)

    def test_small_jitter_above_old_discontinuity_threshold_is_accepted(self):
        from motor_controller.conveyor_index import MM_PER_COUNT
        motion = ConveyorIndex()
        motion.feedback(0, 60.0, True, 0.0)
        motion.feedback(round(0.770 / MM_PER_COUNT), 60.0, True, 0.005)
        self.assertFalse(motion.feedback_fault)
        self.assertIsNone(motion.last_discontinuity)

    def test_single_impossible_encoder_sample_is_discarded_if_next_sample_recovers(self):
        motion = ConveyorIndex()
        motion.feedback(1000000, 10.0, True, 0.0)
        motion.feedback(19000000, 10.0, True, .005)
        self.assertFalse(motion.feedback_fault)
        motion.feedback(1004000, 10.0, True, .010)
        self.assertFalse(motion.feedback_fault)
        self.assertEqual(motion.position_counts, 1004000)
        self.assertIsNone(motion.suspect_discontinuity)
