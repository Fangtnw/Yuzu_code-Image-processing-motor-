# Motor 6 encoder feedback incident

Date: 2026-11-03

## Summary

Motor 6 indexing and short CSP jogs repeatedly enter `interrupted` with an
encoder-discontinuity alert. The reported position jumps are physically
impossible, including jumps of roughly 4.8 m in 5 ms while the conveyor GUI
shows 0 mm/s and a small remaining command error. Restarting only the GUI
does not clear the guard fault. Restarting the EtherCAT/ROS controller has
also been followed by the same alert during another jog.

The encoder-travel number in the GUI (for example, 2368 to 2407 mm) is an
absolute accumulated position conversion. An increase of about 140 mm per
fixture index is expected; this display itself is not evidence of drift and
resetting the logical fixture reference does not zero the encoder.

## Recorded observations

- Guard parameters: `max_rpm=60`, `max_acceleration_rpm_s=120`, command
  watchdog 0.5 s. GUI Acc/Dec fields were 120/120 rpm/s.
- Original index request used about 133.83 mm/s (60 rpm). The GUI and full
  sequence now default to 100 mm/s (about 44.83 rpm); the guard speed cap and
  61 rpm overspeed trip remain unchanged.
- First encoder alert: `+0.770 mm in 5.0 ms; allowed 0.768 mm`; about 69 mm
  remained in the index. This was only 0.002 mm beyond the old continuity
  threshold.
- A separate alert measured `137.18 mm/s` against a `136.06 mm/s` overspeed
  trip, with 67.72 mm remaining. This is a genuine reported-velocity trip
  distinct from the encoder-discontinuity alerts.
- Later alerts: `-4817.383 mm in 5.1 ms; allowed 0.878 mm`, then
  `+4817.444 mm in 5.0 ms`, then `+4817.828 mm in 5.0 ms; allowed 0.869 mm`.
  The last screenshot showed actual speed 0.00 mm/s and only -9.0117 mm
  remaining. Repeated positive and negative jumps of this size suggest a
  single bad position sample or data-path issue, but this has not been proven.
- The 4817 mm jump is approximately 18 million encoder counts using the
  configured scale (`MM_PER_COUNT` about 0.00026766 mm/count). It is not a
  normal 140 mm fixture step.
- On a latched `feedback_fault`, Resume is disabled. Restarting the GUI alone
  cannot clear the guard's fault state.

## Current guard change

`motor_controller/conveyor_index.py` now uses the configured maximum speed
and a 0.2 mm margin for its discontinuity allowance. It also rejects one
impossible position sample and checks the next sample against the last trusted
position. If the next sample returns to a plausible continuation, the suspect
sample is discarded. If the position remains offset, the guard latches the
discontinuity fault. This change is intended to distinguish one corrupt PDO
sample from a persistent encoder position change; it has not yet been
validated on the machine. The separate velocity overspeed check is unchanged.

The GUI default index request is 100 mm/s with 120/120 rpm/s Acc/Dec.
Resuming an interrupted target accepts the currently selected Manual control
speed and ramps while retaining the original absolute target. A latched
encoder fault still cannot be resumed.

## Next investigation

Before another production index, inspect the raw Motor 6 actual-position PDO
and the `motor6_position` ROS state interface while stopped and during a
small supervised jog. Compare consecutive raw counts, source timestamps,
measured velocity, commanded trajectory, and the new `last_discontinuity`
status fields (`delta_counts`, `delta_mm`, `elapsed_s`, `allowed_mm`,
`reported_velocity_rpm`, `command_velocity_rpm`). Check the axis/slave PDO
assignment and feedback wiring against the installed Motor 6 controller.
Do not change the encoder scale or disable the overspeed check without
matching raw PDO evidence. After any latched fault, inspect the mechanism,
restart the guard, align a fixture mark, and establish a new logical
reference only when stopped and the feedback is stable.

## Relevant files

- `motor_controller/conveyor_index.py`: encoder unwrapping, plausibility
  filter, overspeed/settle rules, and diagnostic fields.
- `motor_controller/azd3a_motor6_conveyor_guard.py`: PDO feedback ingestion,
  status topic, CSP target output, and runtime parameters.
- `config/azd3a_motor1_motor2_slave.yaml`: Motor 6 position and velocity PDO
  indices/scales (`0x7064`, `0x706c`).
- `launch/yuzu_peeler.launch.py`: integrated guard parameters.
- `MOTOR6_INDEXING.md`: indexing behavior and previous calibration evidence.
