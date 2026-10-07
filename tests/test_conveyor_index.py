"""Behavioral motion tests: no ROS, EtherCAT, or physical movement."""

from collections import deque
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "motion_cmd"))
from motor_controller.conveyor_index import (  # noqa: E402
    ConveyorIndex, MM_PER_COUNT, MM_S_PER_RPM, NOMINAL_PITCH_MM,
)


class ConveyorPlant:
    """Quantized encoder and velocity PDOs with configurable transport delays."""
    def __init__(self, origin=1_000_000, drive_delay=0, feedback_delay=0, gui_phase=0):
        self.motion = ConveyorIndex()
        self.time = 0.0
        self.position = float(origin)
        self.velocity = 0.0
        self.commands = deque([0.0] * drive_delay)
        self.samples = deque()
        self.feedback_delay = feedback_delay
        self.peak_rpm = 0.0
        self.origin = origin
        self.ticks = 0
        self.gui_phase = gui_phase
        for _ in range(100):
            self.tick()

    def tick(self, heartbeat=True, feedback=True, enabled=True, stalled=False, dt=0.005):
        self.time += dt
        self.ticks += 1
        sample = (round(self.position + 2**31) % 2**32 - 2**31, self.velocity, self.time)
        self.samples.append(sample)
        if len(self.samples) > self.feedback_delay:
            raw, velocity, stamp = self.samples.popleft()
            if feedback:
                self.motion.feedback(raw, velocity, enabled, stamp)
        if heartbeat and self.ticks % 20 == self.gui_phase:
            self.motion.heartbeat(self.time)
        output = self.motion.update(self.time)
        # Drive target velocity is an integer number of encoder counts/second.
        self.commands.append(int(output * 500_000 / 60.0) * 60.0 / 500_000)
        previous = self.velocity
        command = self.commands.popleft()
        self.velocity = 0.0 if stalled else command
        self.peak_rpm = max(self.peak_rpm, abs(self.velocity))
        self.position += (previous + self.velocity) * 0.5 * MM_S_PER_RPM * dt / MM_PER_COUNT

    def request(self, direction=1, rpm=48.0):
        index = self.motion.index if self.motion.origin_counts is not None else 0
        self.motion.request_index(index + direction, rpm, 96.0, 96.0, self.time)

    def finish(self, **kwargs):
        for _ in range(3000):
            self.tick(**kwargs)
            if self.motion.state != "indexing":
                break
        if self.motion.state != "ready":
            raise AssertionError(self.motion.status(self.time))


class ConveyorIndexTests(unittest.TestCase):
    def test_required_step_reaches_speed_and_settles_at_target(self):
        plant = ConveyorPlant()
        plant.request()
        plant.finish()
        travel = (plant.position - plant.origin) * MM_PER_COUNT
        self.assertAlmostEqual(travel, 140.825, delta=0.0201)
        self.assertAlmostEqual(plant.peak_rpm, 48.0, places=5)
        self.assertEqual(plant.velocity, 0.0)
        self.assertEqual(plant.motion.index, 1)

    def test_error_does_not_accumulate_over_ten_complete_belt_circuits(self):
        plant = ConveyorPlant(drive_delay=2, feedback_delay=2)
        for index in range(1, 121):
            plant.request()
            plant.finish()
            global_error = (plant.position - plant.origin) * MM_PER_COUNT - index * 140.825
            self.assertLessEqual(abs(global_error), 0.0201, (index, global_error))
            self.assertAlmostEqual(
                (plant.motion.target_counts - plant.origin) * MM_PER_COUNT,
                index * 140.825, delta=MM_PER_COUNT / 2,
            )

    def test_forward_reverse_returns_to_original_reference(self):
        plant = ConveyorPlant()
        for direction in (1, 1, 1, -1, -1, -1, -1, 1):
            plant.request(direction)
            plant.finish()
        self.assertEqual(plant.motion.index, 0)
        self.assertEqual(plant.motion.target_counts, plant.origin)
        self.assertLessEqual(abs(plant.position - plant.origin) * MM_PER_COUNT, 0.0201)

    def test_feedback_and_drive_delay_do_not_change_endpoint(self):
        for drive_delay, feedback_delay in ((0, 0), (2, 1), (4, 3), (6, 4)):
            with self.subTest(drive_delay=drive_delay, feedback_delay=feedback_delay):
                plant = ConveyorPlant(drive_delay=drive_delay, feedback_delay=feedback_delay)
                plant.request()
                plant.finish()
                self.assertAlmostEqual((plant.position - plant.origin) * MM_PER_COUNT,
                                       140.825, delta=0.0201)

    def test_gui_poll_phase_does_not_decide_braking_or_endpoint(self):
        for phase in range(20):
            plant = ConveyorPlant(gui_phase=phase)
            plant.request()
            plant.finish()
            self.assertAlmostEqual((plant.position - plant.origin) * MM_PER_COUNT,
                                   140.825, delta=0.0201)

    def test_signed_encoder_rollover_in_each_direction(self):
        for origin, direction in ((2**31 - 10_000, 1), (-2**31 + 10_000, -1)):
            plant = ConveyorPlant(origin)
            plant.request(direction)
            plant.finish()
            self.assertAlmostEqual((plant.position - origin) * MM_PER_COUNT,
                                   direction * 140.825, delta=0.0201)

    def test_stop_preserves_target_and_requires_explicit_resume(self):
        plant = ConveyorPlant()
        plant.request()
        for _ in range(120):
            plant.tick()
        target = plant.motion.target_counts
        plant.motion.stop("operator stop")
        for _ in range(200):
            plant.tick()
        self.assertEqual(plant.motion.state, "interrupted")
        self.assertEqual(plant.motion.target_counts, target)
        with self.assertRaises(ValueError):
            plant.motion.request_index(1, 48., 96., 96., plant.time)
        plant.motion.resume(plant.time)
        plant.finish()
        self.assertEqual(plant.motion.target_counts, target)
        plant.request()
        plant.finish()
        self.assertAlmostEqual((plant.position - plant.origin) * MM_PER_COUNT,
                               2 * 140.825, delta=0.0201)

    def test_watchdog_feedback_loss_and_disabled_drive_stop_without_completing(self):
        for failure in (dict(heartbeat=False), dict(feedback=False), dict(enabled=False)):
            with self.subTest(failure=failure):
                plant = ConveyorPlant()
                plant.request()
                for _ in range(120):
                    plant.tick()
                for _ in range(240):
                    plant.tick(**failure)
                self.assertEqual(plant.motion.state, "interrupted")
                self.assertEqual(plant.motion.index, 0)
                self.assertEqual(plant.motion.output_rpm, 0.0)
                self.assertEqual(plant.velocity, 0.0)

    def test_stall_and_missed_control_deadline_never_report_completion(self):
        plant = ConveyorPlant()
        plant.request()
        for _ in range(2000):
            plant.tick(stalled=True)
        self.assertEqual(plant.motion.state, "interrupted")
        self.assertIn("timeout", plant.motion.message)
        plant = ConveyorPlant()
        plant.request()
        plant.tick(dt=0.06)
        self.assertEqual(plant.motion.state, "interrupted")
        self.assertIn("deadline", plant.motion.message)

    def test_duplicate_index_and_busy_rejection_do_not_advance_grid(self):
        plant = ConveyorPlant()
        plant.request()
        target = plant.motion.target_counts
        plant.motion.request_index(1, 48., 96., 96., plant.time)
        self.assertEqual(plant.motion.target_counts, target)
        with self.assertRaises(ValueError):
            plant.motion.request_index(2, 48., 96., 96., plant.time)
        plant.finish()
        plant.motion.request_index(1, 48., 96., 96., plant.time)
        self.assertEqual(plant.motion.state, "ready")
        self.assertEqual(plant.motion.index, 1)

    def test_bad_parameters_and_nonfinite_commands_are_rejected(self):
        for kwargs in (dict(max_rpm=60), dict(max_ramp=float("nan")),
                       dict(command_timeout=1.0), dict(loop_counts=0), dict(tolerance_mm=1)):
            with self.assertRaises(ValueError):
                ConveyorIndex(**kwargs)
        plant = ConveyorPlant()
        for args in ((1, 0, 96, 96), (1, 49, 96, 96), (1, 48, 1, 96),
                     (1, 48, 96, 1), (1, float("nan"), 96, 96), (0.5, 48, 96, 96)):
            with self.assertRaises(ValueError):
                plant.motion.request_index(*args, plant.time)
        self.assertIsNone(plant.motion.origin_counts)

    def test_manual_motion_and_new_session_require_new_grid(self):
        plant = ConveyorPlant()
        plant.request()
        plant.finish()
        plant.motion.manual(5., 96., 96., plant.time)
        self.assertIsNone(plant.motion.origin_counts)
        for _ in range(40):
            plant.motion.manual(5., 96., 96., plant.time)
            plant.tick()
        plant.motion.manual(0., 96., 96., plant.time)
        for _ in range(100):
            plant.tick()
        plant.request()
        origin = plant.motion.origin_counts
        plant.finish()
        self.assertAlmostEqual((plant.position - origin) * MM_PER_COUNT, NOMINAL_PITCH_MM, delta=0.0201)
        self.assertIsNone(ConveyorIndex().origin_counts)

    def test_encoder_discontinuity_and_post_completion_drift_block_next_step(self):
        plant = ConveyorPlant()
        plant.request()
        plant.motion.feedback(1_000_000_000, 0., True, plant.time + .005)
        plant.tick()
        self.assertEqual(plant.motion.state, "interrupted")
        plant = ConveyorPlant()
        plant.request()
        plant.finish()
        plant.position += .04 / MM_PER_COUNT
        plant.tick()
        # Move farther than the endpoint's small positive remaining error.
        plant.position += .04 / MM_PER_COUNT
        plant.tick()
        self.assertEqual(plant.motion.state, "interrupted")
        with self.assertRaises(ValueError):
            plant.request()


if __name__ == "__main__":
    unittest.main()
