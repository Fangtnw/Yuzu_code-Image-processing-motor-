# Motor 6 fixture indexing investigation — 2026-10-07

## Current trial calibration

Subsequent repeated measurements at 30 and 90 mm/s showed 142 mm physical
travel for 140 mm displayed travel. TRAVEL_CALIBRATION=142/140 now multiplies
the provisional transmission conversion for both distance and speed. This is
an empirical correction, not a confirmed timing-pulley tooth ratio. Revalidate
physical travel after applying it.

The current requirement is 140 mm per step (1680 mm / 12), at 90 mm/s.
The motor/roller timing transmission is provisionally 1.4, inferred from
28/20 mm pulley outer diameters and 41.85 mm measured travel for a displayed
30 mm. Confirm tooth counts and physical travel before accepting accuracy.
This gives 3789.403407 counts/mm, 530516 counts/step, and 40.925557 output rpm
at 90 mm/s. Older calculations below describe the previous 1:1 assumption.

## Current requirement and implementation

Motor 6 uses CiA-402 cyclic synchronous position (CSP, mode 8). The guard
publishes an absolute output-shaft position target every 5 ms, so the drive
closes the position loop and holds the endpoint instead of relying on a late
CSV velocity stop.

The GUI also provides a bounded relative CSP jog of up to 200 mm. A jog clears
the old fixture grid when it completes; align the visible mark and use the
reference command before indexing again.

The operator confirmed 12 fixtures and requested **140.825 mm per step** at
80% of Motor 6's documented 60 rpm maximum: **48 rpm**, nominally
75.3982 mm/s using the existing 30 mm pulley conversion. Acceleration and
deceleration remain 96 rpm/s. No sensor is installed. The current scope is
accurate encoder-based stepping; no fixture-mark confirmation workflow was added.

`conveyor_index.py` generates absolute count targets from the first step's
stopped position. Each target is `origin + round(index * pitch_counts)`;
rounding and previous endpoint errors are not accumulated. The first step
captures its reference automatically after fresh, enabled, stationary feedback.
Subsequent forward/reverse steps select the adjacent index on the same grid.
Manual speed operation deliberately clears the grid; a backend restart also
starts a new grid. The GUI labels these behaviors explicitly.

The independent 200 Hz guard generates a smooth 200 Hz target trajectory under
velocity/acceleration limits and then holds the absolute target. Completion requires error within
0.02 mm of encoder-derived travel, zero commanded speed and settled measured
speed (at most 0.02 mm/s). It waits 0.15 s for stationary feedback, then
0.15 s continuously in the completion condition. This is a software criterion,
not a manufacturer specification for physical fixture accuracy.

Stops retain the target for explicit **Resume interrupted step**. Stale
feedback, a disabled drive, lost command heartbeat, a missed control deadline,
or failure to settle before the motion timeout interrupts the index. A new
step is rejected while busy or interrupted. The operation sequence waits for
backend completion before enabling the placement stage.

## Command and feedback interfaces

- Existing `/motor6_conveyor/commands_rpm`: `[rpm]` or
  `[rpm, acceleration, deceleration]` for manual speed; zero always requests
  a controlled stop. Nonzero manual commands cannot override an active index.
- `/motor6_conveyor/index_request`: `std_msgs/String` JSON with `id` (unique
  request ID), `session` (current backend boot ID), `reference_id`, `sent_ns`
  (ROS-clock nanoseconds), and `op` (`index`, `resume`, or `reference`). An
  `index` request also supplies integer `index`, `rpm`, `acceleration`, and
  `deceleration`. Only an adjacent target is accepted. Repeated request IDs
  do not restart a move; requests predating a stop or older than 0.5 s are
  rejected. An explicit `reference` operation resets the grid while stopped.
- `/motor6_conveyor/index_heartbeat`: `std_msgs/String` containing the accepted
  request ID, refreshed during an index. The watchdog remains 0.5 s.
- `/motor6_conveyor/index_status`: `std_msgs/String` JSON with request
  acknowledgement, state, current/target index, pitch, remaining error,
  stationary/fresh flags and raw unwrapped origin/position/target counts.
- A complete `/dynamic_joint_states` sample supplies position, velocity and
  Motor 6 status together. Source timestamps must be fresh within 0.1 s;
  status-only, invalid, duplicate or old samples cannot refresh position.
  Combined feedback uses `motor4_joint/motor6_position`, `velocity` and
  `motor6_status`. The standalone launch uses `motor6_joint/position`,
  `velocity` and the newly exported `motor6_status`.

Guard startup parameter `index_loop_counts` defaults to the count equivalent
of `12 * 140.825 mm`, using the existing 30 mm pulley scale. A measured full
belt-circuit count value can replace it during later calibration; keep the
full value internally. `index_tolerance_mm` defaults to 0.02 and permits only
one encoder count through 0.1 mm. GUI pitch fields are read-only backend values.
Do not change these parameters during a move; they are read at startup.

## Specification check

The project hardware record identifies `SVKA-150-795-25-...` and belt
`HBLTWH150-1.68`. The archived [SVKA catalog](../vendor/misumi/SVKA_catalog_2019_pages_1254-1255.pdf)
labels L as pulley-center distance. Its drawing uses a 30 mm pulley and a
0.9 mm H belt, giving 31.8 mm over the belt; it explicitly cautions that
dimensions depend on belt specification. The installed conveyor's W belt
must not automatically inherit the drawing's H-belt thickness.
[Official catalog](https://us.misumi-ec.com/pdf/fa/2019/2019_US_1254.pdf).

For an ideal two-pulley outer-surface path, assuming twelve fixtures:

```text
Drawing-based path: 2 * 795 + pi * 31.8 = 1689.902646 mm
Divide by 12:                            140.825221 mm
Rounded proposed pitch:                 140.8 mm
12 * 140.8:                             1689.6 mm
```

This plausibly explains the proposed number but does not establish the
material spacing of fixtures or encoder travel for a complete belt circuit.
Even this ideal estimate differs from twelve rounded steps by 0.302646 mm.

MISUMI defines belt order length as the inner/back-surface circumference.
Thus `-1.68` denotes nominal 1680 mm inner circumference, not a measured
installed outer-surface loop. Its current Japanese HBLTWH table lists 0.8 mm
thickness, length tolerance +/-10 mm for lengths up to 2000 mm, and permanent
elongation up to 1% as a reference value. The actual supplied belt revision,
tension and installed dimensions still need checking.
[Belt specifications](https://jp.misumi-ec.com/vona2/detail/110302562820/).

The official SVKA replacement-belt formula is `(2L + 97) / 1.002 / 1000`
metres, with rounding to two decimal places. At L=795 it gives 1.683633 m,
rounding to 1.68 m, consistent with the recorded belt label. This is a belt
ordering calculation, not a precision fixture-indexing specification.
[Belt length calculation](https://us.misumi-ec.com/maker/misumi/mech/product/cvs/calculation/).

## Reproduction and validation ledger

1. Previous GUI stopping checked distance every 100 ms and discarded the
   target at the braking request. An offline simulation executing that stop
   branch and the actual guard ramp produced 140.9947..148.1575 mm for a
   140.8 mm target across GUI phases in both directions, with perfect feedback
   and no slip. Changing only polling to 5 ms yielded 140.9947 mm, showing
   that faster polling alone was insufficient.
2. The PDO scale matches the commissioning record: 500,000 counts/output
   revolution. The old encoder-scale error is not present in this source.
   This does not establish live physical belt calibration.
3. The new tests execute the motion model with integer encoder/velocity PDOs.
   They cover 120 consecutive 140.825 mm steps (ten circuits), reversals,
   both signed encoder rollovers, all 20 phases of a 100 ms GUI refresh,
   drive delays through 30 ms and feedback delays through 20 ms. Settled
   error stays within 0.0201 mm against the original ideal target grid
   (the extra 0.0001 mm in the physical simulation comparison accommodates
   half-count quantization). Peak speed reaches 48 rpm.
4. Callback/GUI tests cover missing/stale/invalid feedback, drive faults,
   watchdog expiry, stalls, missed deadlines, duplicate commands, a command
   arriving after Stop, acknowledgements, explicit resume, and blocking the
   placement stage until its matching index is complete.
5. All 52 repository tests pass; Python compilation, combined/standalone
   xacro expansion and an isolated `motor_controller` package build pass.
   ROS modules/launch descriptions are checked without starting hardware.

Run the offline checks from the repository root:

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
```

Tests establish software behavior under the stated plant model, not physical
belt accuracy. No live motor command or live-workspace rebuild was issued.

## Physical validation still required

1. With the confirmed 12 fixtures, measure the installed belt circuit at normal
   tension. Record encoder travel between successive returns of the same
   marked belt reference to a fixed station; repeat to assess variation.
2. The implemented grid uses equally spaced targets from one captured
   reference: `origin_counts + round(index * measured_loop_counts / count)`.
   Never start the next target from the previous measured stop, and never
   repeatedly add a separately rounded count increment.
3. Measure a marked 140.825 mm move and repeated full circuits after rebuilding
   and restarting the control session. Check endpoint error, stopping, direction
   and alarms under the intended load. The former 100 ms GUI loop is no longer
   involved in stopping decisions, but belt scaling still requires measurement.
4. Use a belt reference sensor to correct registration each circuit if
   cumulative physical drift is unacceptable. Detect each fixture at the
   station if individual fixture placement/spacing also needs correction.

An absolute encoder target grid prevents software accumulation; it cannot
detect flat-belt slip or stretch. A motor stop-accuracy specification is not
a complete conveyor positioning tolerance. Acceptance needs physical station
measurements over repeated full circuits under the intended fixture load.
