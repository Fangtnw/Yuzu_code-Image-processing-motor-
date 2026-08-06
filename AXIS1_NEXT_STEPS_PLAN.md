# Axis 1 Next Steps Plan

> **For Ubuntu Codex:** Read this after `progress.md` and
> `motion_cmd/AZD3A_HARDWARE.md`. The goal is to make Axis 1 repeatable,
> measured, and safe before adding Axis 2 or Axis 3.

## Current State

- EtherCAT discovery works.
- PDO/SDO read-only communication works.
- ROS 2 feedback through `/joint_states` works.
- Axis 1 tiny commanded motion has been physically confirmed.
- Axis 1 is currently wired to `DR28T1A03-AZAKR`, a 1 mm-lead, 30 mm-stroke
  linear actuator. This model is the machine Motor 3/4 actuator type, but it
  is connected as AZD3A Axis 1 for bring-up validation.
- Verified Axis 1 scaling is `10,000,000 counts/m`.

Do not treat the first movement as complete validation. The next milestone is
repeatable motion with captured feedback evidence, followed by explicitly
preparing the Axis 2 and Axis 3 control-interface requirements.

## Safety Boundary

- Keep only Axis 1 under motion test.
- Keep the physical power cutoff reachable.
- Keep motion inside a conservative range, initially `0 mm` to `5 mm`.
- Use absolute position commands, not accumulating relative moves.
- Do not add Axis 2/3 motion until Axis 1 direction, feedback, stop behavior,
  and limits are repeatable.
- Axis 2/3 should be added as read-only feedback first. Do not enable their
  motion controllers until the intended rotary interface, scaling, and safe
  speed/angle limits are confirmed.

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

## Task 5: Check Requirements for All Three Control Interfaces

Current repo state:

- `motion_cmd/urdf/azd3a_axis1_feedback.urdf.xacro` defines only
  `axis1_joint`.
- `motion_cmd/urdf/azd3a_axis1_tiny_move.urdf.xacro` defines only
  `axis1_joint`.
- `motion_cmd/config/azd3a_ked_cia402_slave.yaml` maps only Axis 1 PDO
  `0x1600` / `0x1A00`.
- `motion_cmd/config/azd3a_axis1_tiny_move_controllers.yaml` exposes only
  `axis1_position_controller`.

Requirement before all 3 axes can have ROS control interfaces:

- [ ] Confirm Axis 2 and Axis 3 should be exposed as rotary joints:
      `axis2_joint` and `axis3_joint`.
- [ ] Do not map final machine motor numbers directly to current AZD3A axis
      numbers without verifying the actual wiring and connected model.
- [ ] Do not assume `position` is the only rotary command interface. The vendor
      manual supports position modes and velocity modes:
      PP=`1`, PV=`3`, HM=`6`, CSP=`8`, CSV=`9`.
- [ ] Treat Axis 1 as a position interface first because the currently wired
      DR28 actuator is the tested 30 mm linear actuator.
- [ ] Treat Axis 2/3 as rotary interfaces that may need velocity/rpm
      start-stop control first, with optional angle/position control only when
      the sequence needs indexing or orbit positioning.
- [ ] Use the corrected Axis 2 / machine Motor 2 model:
      `AZM46AK-FC7.2UA`, intended for about `200-250 rpm` spin/stop.
- [ ] Keep `AZM46AK-FC20DA` scaling notes assigned to Axis 3 / machine Motor 5
      only, unless physical wiring proves otherwise.
- [ ] Confirm Axis 2 PDO objects:
      Controlword `0x6840`, Target position `0x687A`, Mode command `0x6860`,
      Target velocity `0x68FF`, Statusword `0x6841`, Actual position `0x6864`,
      Mode display `0x6861`.
- [ ] Confirm Axis 3 PDO objects:
      Controlword `0x7040`, Target position `0x707A`, Mode command `0x7060`,
      Target velocity `0x70FF`, Statusword `0x7041`, Actual position `0x7064`,
      Mode display `0x7061`.
- [ ] Confirm Axis 2/3 rotary scaling from live MEXE02/SDO settings before
      enabling motion. Axis 2 uses `AZM46AK-FC7.2UA`, so do not copy the
      `FC20DA` 20:1 reference scaling onto it. The current hardware note gives
      the `3183.098862 counts/rad` and `0.000314159265 rad/count` reference
      value only for Axis 3 / `AZM46AK-FC20DA`, assuming the drive resolution
      is `1000 P/R` with a `20:1` gearbox.
- [ ] Confirm the velocity unit conversion before any rpm command. The AZD3A
      object name is Target velocity `[Hz]`; the operator-facing command may be
      rpm, but the EtherCAT PDO value must be converted to the drive's expected
      unit for the selected mode.
- [ ] Map the current sequence intent before choosing controllers:
      Motor 2 is spin/stop at about `200-250 rpm`; Motor 5 is rotational motion
      around `20 rpm`; linear/feed axes remain position plus speed style
      commands.
- [ ] Decide whether the next implementation should use one combined
      three-axis URDF/config or separate staged files:
      `axis2_feedback`, `axis2_tiny_move`, `axis3_feedback`, `axis3_tiny_move`.

Recommended staged implementation:

1. Add Axis 2/3 to a feedback-only launch first.
2. Verify `/joint_states` includes:

   ```text
   axis1_joint
   axis2_joint
   axis3_joint
   ```

3. Confirm Axis 2/3 feedback changes only when the physical motors are moved or
   commanded later.
4. Add Axis 2 tiny rotary-speed test after velocity scaling and rpm limits are
   confirmed.
5. Add Axis 3 tiny rotary-speed or angle test only after its role is confirmed
   from the mechanism/sequence.

Do not use the Axis 2/3 reference scaling for real motion until the live drive
settings confirm it. Wrong rotary position or velocity scaling can command a
much larger move or speed than intended.

## Task 6: Decide Next Hardware Target

After Axis 1 repeatability is documented:

1. All-three-axis interface requirement check.
2. Axis 2/3 read-only feedback.
3. Axis 2 tiny motion.
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
- Axis 2/3 control-interface requirements were checked before editing their
  motion configs.
