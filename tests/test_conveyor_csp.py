"""CSP trajectory regression checks without driving hardware."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'motion_cmd'))
from motor_controller.conveyor_index import ConveyorIndex, MM_PER_COUNT, OPERATING_MAX_RPM


class CspTests(unittest.TestCase):
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
            motion.jog(35.995, OPERATING_MAX_RPM, 96, 96, .2)
        else:
            motion.request_index(1, OPERATING_MAX_RPM, 96, 96, .2)
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
        self.assertTrue(motion.feedback_fault)
        motion.feedback(0, 0, True, .010)
        self.assertFalse(motion.fresh(.010))
        self.assertFalse(motion.enabled)
