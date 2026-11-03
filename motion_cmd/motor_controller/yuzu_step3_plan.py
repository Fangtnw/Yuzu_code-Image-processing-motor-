"""Complete, prevalidated Motor 3/4/5 contour peeling action plan."""

from dataclasses import dataclass

from .yuzu_contour_feed_preview import build_rows


@dataclass(frozen=True)
class Step3Action:
    phase: str
    motor5_deg: float
    motor3_mm: float
    motor4_mm: float
    command_motors: tuple


def build_step3_actions(data, motor3_center_mm=35.0, motor4_center_mm=35.0,
                        source_inset_mm=3.0, peel_depth_mm=0.0):
    """Validate all rays/targets, then plan standby, sweep, release, return."""
    rows = build_rows(data, motor3_center_mm, motor4_center_mm,
                      source_inset_mm, peel_depth_mm)
    if len(rows) != 21 or rows[0]["motor5_deg"] != 90 or rows[-1]["motor5_deg"] != -10:
        raise ValueError("Unexpected Motor 5 orbit waypoint sequence")

    actions = [Step3Action("standby", 90.0, 0.0, 0.0, (5,))]
    for index, row in enumerate(rows):
        actions.append(Step3Action(
            "feed_start" if index == 0 else "peel_sweep",
            row["motor5_deg"], row["motor3_target_mm"], row["motor4_target_mm"],
            (3, 4) if index == 0 else (3, 4, 5),
        ))
    actions.append(Step3Action("release", -10.0, 0.0, 0.0, (3, 4)))
    actions.append(Step3Action("return_to_origin", 0.0, 0.0, 0.0, (5,)))
    return tuple(actions)


def interpolated_feed(actions, motor5_deg):
    """Linearly interpolate both slide targets between frozen 5° sweep samples."""
    if not -10.0 <= motor5_deg <= 90.0:
        raise ValueError("Motor 5 angle is outside the peel sweep")
    samples = [action for action in actions if action.phase in ("feed_start", "peel_sweep")]
    for upper, lower in zip(samples, samples[1:]):
        if lower.motor5_deg <= motor5_deg <= upper.motor5_deg:
            fraction = (upper.motor5_deg - motor5_deg) / 5.0
            return (round(upper.motor3_mm + fraction * (lower.motor3_mm - upper.motor3_mm), 4),
                    round(upper.motor4_mm + fraction * (lower.motor4_mm - upper.motor4_mm), 4))
    return samples[-1].motor3_mm, samples[-1].motor4_mm
