# Axis 1 Next Steps Plan

> **For Ubuntu Codex:** Read this after `progress.md` and
> `motion_cmd/AZD3A_HARDWARE.md`. The goal is to make Axis 1 repeatable,
> measured, and safe before adding Axis 2 or Axis 3.

## Current State

- EtherCAT discovery works.
- PDO/SDO read-only communication works.
- ROS 2 feedback through `/joint_states` works.
- Axis 1 tiny commanded motion has been physically confirmed.
- Axis 1 is `DR28T1A03-AZAKR`, a 1 mm-lead, 30 mm-stroke linear actuator.
- Verified Axis 1 scaling is `10,000,000 counts/m`.

Do not treat the first movement as complete validation. The next milestone is
repeatable motion with captured feedback evidence.

## Safety Boundary

- Keep only Axis 1 under motion test.
- Keep the physical power cutoff reachable.
- Keep motion inside a conservative range, initially `0 mm` to `5 mm`.
- Use absolute position commands, not accumulating relative moves.
- Do not add Axis 2/3 motion until Axis 1 direction, feedback, stop behavior,
  and limits are repeatable.

## Task 1: Capture Feedback After Motion

- [ ] Start the Axis 1 tiny-motion launch.
- [ ] Send one small absolute command, such as `0.0001 m`.
- [ ] Capture `/joint_states` after motion settles.
- [ ] Record the final position in `progress.md`.
- [ ] State clearly whether the value is measured feedback or only physical
      observation.

Useful commands:

```bash
ros2 launch motor_controller azd3a_axis1_tiny_move.launch.py
ros2 topic pub --once /axis1_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0001]}"
ros2 topic echo /joint_states --once
```

## Task 2: Repeatability Check

Run a small absolute-position sequence and compare physical direction with
`/joint_states`.

- [ ] Command `0.0001 m`.
- [ ] Command `0.0005 m`.
- [ ] Command `0.0010 m`.
- [ ] Command back to `0.0001 m`.
- [ ] Confirm direction is correct.
- [ ] Confirm reported position changes in the expected direction.
- [ ] Record any abnormal sound, alarm, delay, or overshoot.

Use only one command at a time:

```bash
ros2 topic pub --once /axis1_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0005]}"
```

## Task 3: Confirm Software Limits

- [ ] Inspect the active Axis 1 xacro/config limits.
- [ ] Confirm limits are inside the physical 30 mm stroke.
- [ ] Keep the first working limit range conservative, for example `0 mm` to
      `5 mm`.
- [ ] Update the relevant config or `progress.md` if the active limits are not
      obvious.

Important: the drive-level software limits read from SDO are effectively the
full signed 32-bit range, so they do not protect the physical 30 mm actuator.
ROS-side limits and operator discipline are required.

## Task 4: Video Demo Evidence

If the video demo has already been recorded:

- [ ] Add the result to `progress.md`.
- [ ] Record the command or executable used.
- [ ] Record the observed start/end positions if `/joint_states` was captured.
- [ ] Mention that the video proves visible Axis 1 motion, but measured
      convergence still depends on captured feedback.

If the video demo has not been executed yet:

- [ ] Run only when the start position is inside the allowed range.
- [ ] Type the exact confirmation required by the executable.
- [ ] Stop after one bounded demo. Do not loop.

Expected executable:

```bash
ros2 run motor_controller azd3a_axis1_video_demo
```

## Task 5: Decide Next Hardware Target

After Axis 1 repeatability is documented:

1. Axis 2 read-only feedback.
2. Axis 2 tiny motion.
3. Axis 3 read-only feedback.
4. Axis 3 tiny motion.

Do not jump directly to combined three-axis motion. Each axis should first pass
the same sequence: read-only feedback, scaling check, tiny motion, stop/safety
check, documentation update.

## Done Criteria

Axis 1 is ready for the next axis only when:

- `/joint_states` was captured after commanded motion.
- Direction and scaling make sense.
- Several small absolute moves were repeatable.
- Motion stayed inside conservative limits.
- No drive error or unsafe behavior occurred.
- `progress.md` was updated with evidence.
