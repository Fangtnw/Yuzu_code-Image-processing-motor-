"""Feedback-gated target plan for the Motor 5 peeling orbit.

This module plans targets only. The slide depths and live motion coordinator
must be supplied after the Motor 3/4 geometry is calibrated.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class OrbitWaypoint:
    phase: str
    motor5_target_deg: float
    update_slides: bool = False


def orbit_waypoints(clockwise_sign=1, step_deg=5, overshoot_deg=10):
    """Move 90° CW, sweep 100° CCW, retract slides, then home CW.

    ``clockwise_sign`` maps physical CW to the signed Motor 5 guard target.
    The first waypoint requires the operator to have captured zero at the
    side-of-fruit pose. Each following target must settle before advancing.
    """
    if clockwise_sign not in (-1, 1):
        raise ValueError("clockwise_sign must be +1 or -1")
    if isinstance(step_deg, bool) or not isinstance(step_deg, int) or step_deg <= 0:
        raise ValueError("step_deg must be a positive whole number")
    if isinstance(overshoot_deg, bool) or not isinstance(overshoot_deg, int) or overshoot_deg < 0:
        raise ValueError("overshoot_deg must be a nonnegative whole number")
    if 90 % step_deg or overshoot_deg % step_deg:
        raise ValueError("step_deg must divide both 90 and overshoot_deg")
    if 90 + overshoot_deg > 180:
        raise ValueError("orbit exceeds the Motor 5 +/-180 degree guard")

    waypoints = [OrbitWaypoint("side_zero", 0),
                 OrbitWaypoint("standby_cw", clockwise_sign * 90),
                 OrbitWaypoint("slides_feed_in", clockwise_sign * 90, True)]
    for angle in range(90 - step_deg, -overshoot_deg - 1, -step_deg):
        waypoints.append(OrbitWaypoint("peel_sweep_ccw", clockwise_sign * angle, True))
    waypoints.append(OrbitWaypoint("slides_retract", -clockwise_sign * overshoot_deg))
    waypoints.append(OrbitWaypoint("return_to_origin", 0))
    return tuple(waypoints)
