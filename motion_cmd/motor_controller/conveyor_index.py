"""Motor 6 motion model, independent of ROS so indexing can be tested offline.

All index targets share one count origin. The nominal loop is only a geometric
estimate; replace loop_counts with measured belt-circuit travel for calibration.
"""

import math


COUNTS_PER_TURN = 500_000
COUNTS_PER_RAD = COUNTS_PER_TURN / (2.0 * math.pi)
TRANSMISSION_RATIO = 1.4  # Provisional motor pulley / roller pulley tooth ratio.
TRAVEL_CALIBRATION = 142.0 / 140.0  # Repeated physical measurements at 30 and 90 mm/s.
ROLLER_DIAMETER_MM = 30.0
MM_PER_COUNT = math.pi * ROLLER_DIAMETER_MM * TRANSMISSION_RATIO * TRAVEL_CALIBRATION / COUNTS_PER_TURN
MM_S_PER_RPM = math.pi * ROLLER_DIAMETER_MM * TRANSMISSION_RATIO * TRAVEL_CALIBRATION / 60.0
OPERATING_SPEED_MM_S = 90.0
OPERATING_MAX_RPM = OPERATING_SPEED_MM_S / MM_S_PER_RPM
FIXTURE_COUNT = 12
NOMINAL_PITCH_MM = 140.0
NOMINAL_LOOP_MM = NOMINAL_PITCH_MM * FIXTURE_COUNT
FEEDBACK_TIMEOUT_S = 0.1
SETTLE_TIME_S = 0.15
STOP_SPEED_MM_S = 0.02


class ConveyorIndex:
    def __init__(self, max_rpm=OPERATING_MAX_RPM, max_ramp=96.0, command_timeout=0.5,
                 loop_counts=NOMINAL_LOOP_MM / MM_PER_COUNT, tolerance_mm=0.02):
        values = (max_rpm, max_ramp, command_timeout, loop_counts, tolerance_mm)
        if not all(math.isfinite(v) and v > 0.0 for v in values):
            raise ValueError("Conveyor configuration must be finite and positive")
        if max_rpm > OPERATING_MAX_RPM + 1e-9 or max_ramp < max_rpm / 0.5:
            raise ValueError("Conveyor cap is 90 mm/s; ramp must support a 0.5 s stop")
        if command_timeout <= 0.005 or command_timeout > 0.5:
            raise ValueError("Conveyor watchdog must be within (0.005, 0.5] s")
        if not 1.0 <= loop_counts * MM_PER_COUNT / FIXTURE_COUNT <= 1000.0:
            raise ValueError("Calibrated fixture pitch must be within 1..1000 nominal mm")
        if not MM_PER_COUNT <= tolerance_mm <= 0.1:
            raise ValueError("Settling tolerance must be one encoder count..0.1 mm")
        self.max_rpm = max_rpm
        self.max_ramp = max_ramp
        self.command_timeout = command_timeout
        self.loop_counts = loop_counts
        self.tolerance_mm = tolerance_mm
        self.raw_counts = self.position_counts = None
        self.command_counts = None
        self.feedback_time = None
        self.velocity_rpm = 0.0
        self.enabled = False
        self.feedback_fault = False
        self.stationary_since = None
        self.stationary_position_counts = None
        self.origin_counts = self.target_counts = None
        self.jog_target_counts = None
        self.index = self.target_index = 0
        self.reference_id = 0
        self.state = "unreferenced"
        self.message = "Align a fixture and set the conveyor reference"
        self.output_rpm = self.manual_rpm = 0.0
        self.speed_rpm = max_rpm
        self.acceleration = self.deceleration = max_ramp
        self.last_command = self.last_update = self.deadline = None
        self.settle_since = None
        self.last_overspeed = None

    def feedback(self, raw_counts, velocity_rpm, enabled, now):
        if self.feedback_fault:
            return  # A discontinuity requires a backend restart, not a later sample.
        if not all(math.isfinite(v) for v in (raw_counts, velocity_rpm, now)):
            self.stop("Invalid conveyor feedback")
            self.enabled = False
            return
        raw_counts = round(raw_counts)
        if not -(2**31) <= raw_counts < 2**31:
            self.stop("Encoder feedback outside signed 32-bit range")
            self.enabled = False
            return
        if self.feedback_time is not None and now <= self.feedback_time:
            return  # Repeated/out-of-order samples cannot refresh readiness.
        if self.raw_counts is None:
            position = raw_counts
        else:
            elapsed = now - self.feedback_time
            delta = (raw_counts - self.raw_counts + 2**31) % 2**32 - 2**31
            # Unwrap the multi-turn signed PDO at either boundary. A reset or
            # implausible discontinuity requires investigation, not a new origin.
            allowance = 60.0 * MM_S_PER_RPM * elapsed + 0.1
            if allowance >= 2**31 * MM_PER_COUNT or abs(delta) * MM_PER_COUNT > allowance:
                self.stop("Encoder discontinuity; restart and re-reference after inspection")
                self.enabled = False
                self.feedback_fault = True
                return
            position = self.position_counts + delta
        self.raw_counts = raw_counts
        self.position_counts = position
        if self.command_counts is None:
            self.command_counts = position
        self.feedback_time = now
        self.velocity_rpm = velocity_rpm
        self.enabled = bool(enabled)
        # The drive's instantaneous velocity estimate can spike at rest.
        # Require a stable encoder position over the entire settling window,
        # not just a small delta between successive samples (which could
        # wrongly admit continuous slow movement).
        if self.enabled and abs(self.output_rpm) < 1e-9:
            if (self.stationary_since is None or self.stationary_position_counts is None
                    or abs(position - self.stationary_position_counts) > 2):
                self.stationary_since = now
                self.stationary_position_counts = position
        else:
            self.stationary_since = None
            self.stationary_position_counts = None

    def fresh(self, now):
        return (self.enabled and self.feedback_time is not None
                and 0.0 <= now - self.feedback_time <= FEEDBACK_TIMEOUT_S)

    def stopped(self, now):
        return (self.fresh(now) and abs(self.output_rpm) < 1e-9
                and self.stationary_since is not None
                and now - self.stationary_since >= SETTLE_TIME_S)

    def error_mm(self):
        if self.target_counts is None or self.position_counts is None:
            return None
        return (self.target_counts - self.position_counts) * MM_PER_COUNT

    def capture_reference(self, now):
        if self.state in ("indexing", "manual") or not self.stopped(now):
            raise ValueError("Reference capture requires fresh, enabled, stopped feedback")
        self.origin_counts = self.target_counts = self.position_counts
        self.command_counts = self.position_counts
        self.index = self.target_index = 0
        self.reference_id += 1
        self.state = "ready"
        self.message = "Fixture 0 reference captured"

    def profile(self, speed, acceleration, deceleration):
        if not all(math.isfinite(v) and v > 0 for v in (speed, acceleration, deceleration)):
            raise ValueError("Speed and ramps must be finite and positive")
        if speed > self.max_rpm + 1e-9:
            raise ValueError("Speed exceeds configured conveyor cap")
        if max(acceleration, deceleration) > self.max_ramp:
            raise ValueError("Ramp exceeds configured conveyor ceiling")
        if acceleration + 1e-9 < speed / 0.5 or deceleration + 1e-9 < max(speed, abs(self.output_rpm)) / 0.5:
            raise ValueError("Ramps must reach/stop the requested speed within 0.5 s")

    def request_index(self, index, speed, acceleration, deceleration, now):
        if not math.isfinite(index) or int(index) != index or abs(index) > 1_000_000:
            raise ValueError("Index must be an integer within +/-1000000")
        self.profile(speed, acceleration, deceleration)
        if self.origin_counts is None:
            if index not in (-1, 1):
                raise ValueError("The first index must be +1 or -1")
            self.capture_reference(now)
        if index == self.target_index and self.state in ("ready", "indexing"):
            return  # An absolute target is idempotent, not another relative step.
        if self.state != "ready" or not self.stopped(now):
            raise ValueError("Wait for completion, or resume the interrupted target")
        if abs(self.error_mm()) > self.tolerance_mm:
            raise ValueError("Conveyor has moved off its last index; resume that target")
        if abs(index - self.index) != 1:
            raise ValueError("Only one adjacent fixture may be indexed at a time")
        self.speed_rpm, self.acceleration, self.deceleration = speed, acceleration, deceleration
        self.target_index = int(index)
        self.target_counts = self.origin_counts + round(index * self.loop_counts / FIXTURE_COUNT)
        self._begin(now)

    def jog(self, distance_mm, speed, acceleration, deceleration, now):
        """Move a bounded relative distance, then require a new reference."""
        if not math.isfinite(distance_mm) or abs(distance_mm) < MM_PER_COUNT or abs(distance_mm) > 200.0:
            raise ValueError("Jog distance must be between one encoder count and 200 mm")
        self.profile(speed, acceleration, deceleration)
        if self.state in ("indexing", "jogging") or not self.stopped(now):
            raise ValueError("Jog requires fresh, enabled, stopped feedback")
        self.speed_rpm, self.acceleration, self.deceleration = speed, acceleration, deceleration
        self.jog_target_counts = self.position_counts + round(distance_mm / MM_PER_COUNT)
        self.target_counts = self.jog_target_counts
        self.command_counts = self.position_counts
        self.state = "jogging"
        self.last_command = now
        self.settle_since = None
        travel_time = abs(distance_mm) / (speed * MM_S_PER_RPM)
        self.deadline = now + max(5.0, 2.0 * travel_time + 3.0)
        self.message = f"Jogging {distance_mm:+.3f} mm; reset reference at the fixture mark"

    def _begin(self, now):
        if self.command_counts is None:
            self.command_counts = self.position_counts
        self.state = "indexing"
        self.last_command = now
        self.settle_since = None
        travel_time = abs(self.error_mm()) / (self.speed_rpm * MM_S_PER_RPM)
        self.deadline = now + max(5.0, 2.0 * travel_time + 3.0)
        self.message = f"Indexing to fixture {self.target_index % FIXTURE_COUNT} (index {self.target_index})"

    def resume(self, now):
        if self.state != "interrupted" or self.target_counts is None or not self.stopped(now):
            raise ValueError("Resume requires an interrupted target and stopped, fresh feedback")
        if abs(self.error_mm()) > 1000.0:
            raise ValueError("Interrupted target is too far away; realign and set reference")
        self._begin(now)

    def heartbeat(self, now):
        if self.state in ("indexing", "jogging"):
            self.last_command = now

    def manual(self, rpm, acceleration, deceleration, now):
        if not math.isfinite(rpm):
            raise ValueError("RPM must be finite")
        if rpm == 0.0:
            self.stop("Controlled stop requested")
            return
        self.profile(abs(rpm), acceleration, deceleration)
        if self.state == "indexing" or not self.fresh(now):
            raise ValueError("Stop indexing and obtain fresh enabled feedback before manual motion")
        self.origin_counts = self.target_counts = None
        self.state = "manual"
        self.manual_rpm = rpm
        self.acceleration, self.deceleration = acceleration, deceleration
        self.last_command = now
        self.message = "Manual motion; fixture reference cleared"

    def stop(self, reason):
        if self.state in ("indexing", "jogging"):
            self.state = "interrupted"
        elif self.state == "manual":
            self.state = "unreferenced"
        self.manual_rpm = 0.0
        self.settle_since = None
        self.message = reason

    def update(self, now):
        elapsed = 0.005 if self.last_update is None else now - self.last_update
        self.last_update = now
        if self.state in ("indexing", "jogging", "manual"):
            if elapsed < 0 or elapsed > 0.05:
                self.stop("Conveyor control timer missed its deadline")
            elif not self.fresh(now):
                self.stop("Conveyor position/velocity/status feedback lost or drive disabled")
            elif self.last_command is None or now - self.last_command > self.command_timeout:
                self.stop("Conveyor command watchdog expired")
            elif abs(self.velocity_rpm) > self.max_rpm + 1.0:
                self.last_overspeed = {
                    "measured_mm_s": self.velocity_rpm * MM_S_PER_RPM,
                    "command_mm_s": self.output_rpm * MM_S_PER_RPM,
                    "trip_mm_s": (self.max_rpm + 1.0) * MM_S_PER_RPM,
                    "position_counts": self.position_counts,
                    "feedback_age_s": now - self.feedback_time,
                }
                self.stop(f"Measured conveyor speed {abs(self.velocity_rpm) * MM_S_PER_RPM:.2f} mm/s "
                          f"exceeds trip limit {(self.max_rpm + 1.0) * MM_S_PER_RPM:.2f} mm/s")
        dt = max(0.0, min(elapsed, 0.02))
        desired = 0.0
        error = self.error_mm()
        if self.state in ("indexing", "jogging"):
            if now > self.deadline:
                self.stop("Index did not settle before timeout; inspect before resuming")
            elif abs((self.target_counts - self.command_counts) * MM_PER_COUNT) > self.tolerance_mm:
                self.settle_since = None
                # Generate the absolute CSP target from the command trajectory.
                # The drive closes the position loop, so stop timing does not
                # turn into a residual distance error in the belt.
                decel = self.deceleration * MM_S_PER_RPM
                trajectory_error = abs((self.target_counts - self.command_counts) * MM_PER_COUNT)
                braking_speed = math.sqrt(max(0.0, 2.0 * decel * trajectory_error))
                speed = min(self.speed_rpm * MM_S_PER_RPM, braking_speed)
                desired = math.copysign(speed / MM_S_PER_RPM, error)
            elif self.stopped(now) and abs(error) <= self.tolerance_mm:
                if self.settle_since is None:
                    self.settle_since = now
                if now - self.settle_since >= SETTLE_TIME_S:
                    if self.state == "jogging":
                        self.origin_counts = self.target_counts = None
                        self.jog_target_counts = None
                        self.index = self.target_index = 0
                        self.state = "unreferenced"
                        self.message = "Jog complete; align the fixture mark and reset the step reference"
                    else:
                        self.index = self.target_index
                        self.state = "ready"
                        self.message = f"Index {self.index} complete; encoder error {error:+.4f} mm"
            else:
                self.settle_since = None
        elif self.state == "manual":
            desired = self.manual_rpm
        elif self.state == "ready" and self.fresh(now) and abs(error) > self.tolerance_mm:
            self.state = "interrupted"
            self.message = "Conveyor moved off the indexed position; resume or realign"

        # A reversal must decelerate to zero before accelerating the other way.
        if desired * self.output_rpm < 0.0:
            desired = 0.0
        ramp = self.acceleration if abs(desired) > abs(self.output_rpm) else self.deceleration
        difference = desired - self.output_rpm
        self.output_rpm += max(-ramp * dt, min(ramp * dt, difference))
        if self.state in ("indexing", "jogging", "interrupted") and self.command_counts is not None:
            step_counts = self.output_rpm * MM_S_PER_RPM * dt / MM_PER_COUNT
            remaining = self.target_counts - self.command_counts
            if abs(step_counts) >= abs(remaining):
                self.command_counts = self.target_counts
                self.output_rpm = 0.0
            else:
                self.command_counts += step_counts
        return self.output_rpm

    def status(self, now):
        return {
            "state": self.state, "message": self.message,
            "reference_id": self.reference_id, "referenced": self.origin_counts is not None,
            "index": self.index, "target_index": self.target_index,
            "pitch_mm": self.loop_counts * MM_PER_COUNT / FIXTURE_COUNT,
            "loop_counts": self.loop_counts, "error_mm": self.error_mm(),
            "tolerance_mm": self.tolerance_mm, "fresh": self.fresh(now),
            "stopped": self.stopped(now), "velocity_mm_s": self.velocity_rpm * MM_S_PER_RPM,
            "command_velocity_rpm": self.output_rpm,
            "stationary_age_s": (now - self.stationary_since
                                  if self.stationary_since is not None else None),
            "feedback_age_s": (now - self.feedback_time
                                if self.feedback_time is not None else None),
            "enabled": self.enabled, "feedback_fault": self.feedback_fault,
            "last_overspeed": self.last_overspeed,
            "command_position_rad": (self.command_counts / COUNTS_PER_RAD
                                      if self.command_counts is not None else None),
        }
