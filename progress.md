# EtherCAT Bring-up Progress

Last updated: 2026-08-06

## Goal

Bring up one Oriental Motor AZD3A-KED EtherCAT drive on native Ubuntu 22.04
with ROS 2 Humble. First prove discovery and read-only communication; only then
prepare a minimal one-axis `ros2_control` experiment.

## Current Result

The discovery and software-installation phase is complete.

- IgH EtherCAT master 1.6.9 starts successfully.
- `/dev/EtherCAT0` is created with group access.
- The dedicated interface `eno2` is attached and its link is up.
- One slave is detected:

  ```text
  0  0:0  PREOP  +  AZD3A-KED rev0301
  ```

- Traffic is stable: zero lost frames were reported during the check.
- PDO and SDO dictionaries are readable.
- ROS 2 Humble packages build successfully:
  - `ethercat_driver`
  - `ethercat_generic_slave`
  - `ethercat_generic_cia402_drive`
  - `motor_controller`

Axis 1 has now been commanded through ROS 2 and physical motion was confirmed.
The next engineering task is to make that one-axis motion repeatable with
captured feedback evidence before adding Axis 2 or Axis 3 motion.

## Connected Motors and Actuator

| Axis | Connected hardware | Type |
| --- | --- | --- |
| Axis 1 | `DR28T1A03-AZAKR` | Machine Motor 3/4 actuator type, currently wired to Axis 1 for bring-up; 1 mm-lead, 30 mm-stroke linear actuator |
| Axis 2 | `AZM46AK-FC7.2UA` | Machine Motor 2 rotary axis; 200-250 rpm spin/stop target role |
| Axis 3 | `AZM46AK-FC20DA` | Machine Motor 5 indexed rotary axis; rotate 90 degrees CW and return 90 degrees CCW at up to 20 rpm |

The `DR28T1A03-AZAKR` is not throwaway hardware; it is the machine Motor 3/4
actuator type from the mechanism plan. For the current one-drive bring-up, one
of these actuators is wired to AZD3A Axis 1, so the software names it
`axis1_joint`. It is not the final machine Motor 1. It is a guided table
actuator with a 1 mm ball-screw lead, 30 mm stroke, 40 mm/s maximum speed, and
0.001 mm minimum travel amount. Axis 2 and Axis 3 are different rotary
motor/gearbox models, so their scaling must be confirmed separately from
physical labels, MEXE02, and live SDO settings before motion. Complete hardware
and preliminary scaling notes are recorded in `motion_cmd/AZD3A_HARDWARE.md`.

## Verified Read-only Drive State

Axis 1 SDO reads returned:

| Object | Meaning | Value | Interpretation |
| --- | --- | --- | --- |
| `0x603F:00` | Error code | `0x0000` | No drive error |
| `0x6041:00` | Statusword | `0x0270` | Switch-on disabled; not faulted |
| `0x6064:00` | Actual position | `-14` | Position feedback is readable and near zero |
| `0x6502:00` | Supported modes | `0x000001A5` | PP, PV, Homing, CSP, and CSV advertised |

## Ubuntu and IgH Configuration

Tested system:

- Ubuntu 22.04
- ROS 2 Humble
- kernel `6.8.0-124-generic`
- IgH EtherCAT master 1.6.9
- IgH prefix `/opt/etherlab`
- EtherCAT NIC `eno2`
- NIC MAC `a0:36:bc:31:3f:18`

`/etc/sysconfig/ethercat`:

```bash
MASTER0_DEVICE="eno2"
DEVICE_MODULES="generic"
UPDOWN_INTERFACES="eno2"
```

The kernel modules were built from `~/kyutech/ethercat` and installed under:

```text
/lib/modules/6.8.0-124-generic/ethercat/master/ec_master.ko
/lib/modules/6.8.0-124-generic/ethercat/devices/ec_generic.ko
```

The original failure was:

```text
modprobe: ERROR: could not insert 'ec_master': Key was rejected by service
```

Secure Boot validation was disabled through the MOK/shim flow. Verification
reported:

```text
SecureBoot enabled
SecureBoot validation is disabled in shim
```

This allowed the unsigned third-party IgH modules to load. For a production
system, signing the modules with an enrolled Machine Owner Key is preferable.

## Device Permissions

Group-based device access is configured instead of running ROS nodes as root.

`/etc/udev/rules.d/99-EtherCAT.rules`:

```udev
KERNEL=="EtherCAT[0-9]*", GROUP="ethercat", MODE="0660"
```

User `fang` belongs to the `ethercat` group. After a full logout/reboot:

```text
crw-rw---- 1 root ethercat ... /dev/EtherCAT0
```

The `ethercat` CLI then works without `sudo`.

## ROS 2 Driver Workspace

The machine-local workspace is:

```text
~/kyutech/azd3a_ws/
├── src/
│   ├── ethercat_driver_ros2/
│   └── motion_cmd -> main repository/motion_cmd
├── build/
├── install/
└── log/
```

The installed ROS driver is the ICube Robotics
`safety-humble-proto00` branch at commit:

```text
97c6dc3afbb89928901043ad06d1d1e5f7be104f
```

The branch expects EtherLab at `/usr/local/etherlab`, while this computer uses
`/opt/etherlab`. A compatibility link resolves that build-time path:

```text
/usr/local/etherlab -> /opt/etherlab
```

All eight packages built:

```text
ethercat_interface
ethercat_msgs
motor_controller
ethercat_manager
ethercat_driver
ethercat_generic_slave
ethercat_generic_cia402_drive
ethercat_driver_ros2
```

`ethercat_generic_plugins` is a source-directory name, not an exported ROS
package. The repository checker was corrected to check the two real packages:
`ethercat_generic_slave` and `ethercat_generic_cia402_drive`.

## Repository Changes

- Added AZD3A EtherCAT configuration/checking files under `motion_cmd/`.
- Improved IgH configuration hints in
  `motion_cmd/motor_controller/azd3a_ethercat_check.py`.
- Corrected the ROS plugin package checks for the selected Humble branch.
- Added `UBUNTU_ETHERCAT_SETUP_GUIDE.md` for installation, operation,
  troubleshooting, and presentation reference.
- Added `azd3a_ws.repos` to pin the external ROS driver dependency.
- Added the unmodified official rev0301 ESI and multi-axis EtherCAT manual
  under `vendor/oriental_motor/`, with SHA-256 checksums and source URLs.
- Added `motion_cmd/AZD3A_HARDWARE.md` to record the connected device on each
  AZD3A axis and identify remaining scaling checks.

Validation:

```bash
python3 -m unittest tests/test_azd3a_ethercat_config_files.py
```

Result: four tests passed.

## Next Step

The feedback-only ROS launch reaches process startup, but
`ros2_control_node` currently stops before loading the EtherCAT hardware:

```text
undefined symbol: realtime_tools::configure_sched_fifo(int)
```

This is a ROS package binary/version mismatch between `controller_manager` and
`realtime_tools`; it is not an AZD3A alarm, PDO failure, or motor movement.
The hardware plugin never loaded during that launch.

A first ROS-independent experiment tried to enter OP with the `ethercat states`
CLI and then control the CiA 402 state machine through SDO uploads/downloads.
The first SDO read after requesting OP failed with `Input/output error`.
Cleanup succeeded: Axis 1 remained at `-16` steps, error code stayed zero, and
the final Statusword was `0x0270` (switch-on disabled). No motion occurred.
That script was removed because an EtherLab application must activate a domain
and continuously exchange the configured PDOs to maintain a usable OP state;
the one-shot CLI sequence does not do this.

The ROS launch failure was traced to a partial ROS package upgrade:

```text
controller_manager       2.53.1 (2026)
controller_manager_msgs  2.42.0 (2024)
realtime_tools            2.5.0 (2024)
ros2_control             2.42.0 (2024)
ros2_controllers         2.35.0 (2024)
```

The old `librealtime_tools.so` does not export
`realtime_tools::configure_sched_fifo(int)`, which the newer
`ros2_control_node` requires. Upgrade the related binary packages together,
then rebuild the workspace and resume the feedback-only PDO launch. Motor
enable/movement must remain a separate later test.

## Axis 1 Cyclic Feedback Milestone

After upgrading the related ROS Humble binary packages together and rebuilding
`~/kyutech/azd3a_ws`, the feedback-only launch runs successfully:

```bash
ros2 launch motor_controller azd3a_axis1_feedback.launch.py
```

`/joint_states` continuously reports:

```yaml
name:
- axis1_joint
position:
- -1.6e-06
velocity:
- .nan
effort:
- .nan
```

The reported position is in metres. With the configured scaling of
`10,000,000 counts/m`, `-1.6e-06 m` equals `-16` counts. This exactly matches
the earlier SDO reading from `0x6064`, validating the Axis 1 TxPDO mapping and
position scaling end to end:

```text
AZD3A encoder -> TxPDO 0x1A00 / 0x6064 -> EtherLab -> ros2_control
              -> joint_state_broadcaster -> /joint_states
```

The `velocity` and `effort` values are `.nan` by design: the current
feedback-only hardware description exports only the position state interface,
and PDO `0x1A00` does not contain velocity or effort feedback.

No motion command controller is loaded, so this milestone reads feedback
without intentionally enabling or moving Axis 1.

## Prepared Axis 1 Tiny-Motion Test

A separate motion configuration was added and installed:

- `motion_cmd/urdf/azd3a_axis1_tiny_move.urdf.xacro`
- `motion_cmd/config/azd3a_axis1_tiny_move_controllers.yaml`
- `motion_cmd/launch/azd3a_axis1_tiny_move.launch.py`

It uses `ethercat_generic_plugins/EcCiA402Drive` in CSP mode (mode 8) and an
`axis1_position_controller`. Starting this launch automatically transitions
Axis 1 to Operation Enabled and initially holds its last measured position.
This is intentionally separate from the feedback-only launch.

The first planned command is the absolute position `0.0001 m`, equivalent to
1,000 counts or 0.1 mm from coordinate zero. Using an absolute command prevents
the move from accumulating if the message is accidentally sent twice.

The xacro, controller YAML, launch Python, installed data files, and ROS launch
description were validated. The package rebuilt successfully.

### Result

The motion launch was run successfully and the following absolute Axis 1
command was published:

```bash
ros2 topic pub --once \
  /axis1_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0001]}"
```

The user physically confirmed that Axis 1 moved. This validates the complete
command path:

```text
ROS position command -> ros2_control -> EcCiA402Drive
 -> RxPDO 0x1600 (6040/607A/6060) -> AZD3A Axis 1 -> physical actuator motion
```

This is the first confirmed commanded motion of the DR28T1A03-AZAKR through
the new EtherCAT/ROS 2 stack. Capture the final `/joint_states` position before
claiming measured convergence to exactly `0.0001 m`; physical movement itself
is confirmed.

For a clearly visible progress video, the bounded
`azd3a_axis1_video_demo` executable was added. It uses live joint feedback,
requires the start position to be between `-0.1 mm` and `6 mm`, and requires
the exact typed confirmation `VIDEO MOVE`. It performs one slow absolute
round trip from the current position to `5 mm`, then back to `1 mm`, at about
`0.5 mm/s`. It does not loop. The node passed syntax/tests, was rebuilt, and
is installed, but its video motion has not yet been executed.

## Axis 1 Measured Motion Evidence

Task 1 of `AXIS1_NEXT_STEPS_PLAN.md` is complete. During the first attempt,
the drive entered fault with Statusword `0x0238`. After stopping ROS, SDO
`0x603F:00` reported `0xFF34`. The official manual identifies alarm `34h` as
Command pulse error.

The CSP configuration had been running in EtherCAT Free Run mode. Enabling the
ESI-specified Distributed Clock setting `assign_activate: 0x0300` initially
left the drive unable to reach OP because the previous 100 Hz rate meant a
10 ms Sync0 cycle. AZD3A supports DC cycles of 0.5 ms or 1 through 8 ms, so the
hardware and controller rates were changed to 200 Hz (5 ms). After rebuilding,
the launch reported:

```text
Domain: WC 3
Master AL states: 0x08
Slave: State 0x08
STATE: Operation Enabled with status word :567
```

No subsequent fault was observed. An absolute target of `0.0001 m` was sent,
and settled `/joint_states` feedback reported:

```yaml
name:
- axis1_joint
position:
- 9.999999999999999e-05
velocity:
- .nan
effort:
- .nan
```

This is measured position feedback, not only visual observation. The reported
value is the floating-point representation of `0.0001 m`, equal to `0.1 mm`
or 1,000 drive counts with the verified Axis 1 scaling. This validates
commanded and measured convergence for the first Task 1 target.

## Axis 1 Repeatability Sequence

Task 2 measured feedback was captured for four consecutive absolute targets:

| Step | Command | Settled `/joint_states` position |
| ---: | ---: | ---: |
| 1 | `0.0001 m` (0.1 mm) | `9.999999999999999e-05 m` |
| 2 | `0.0005 m` (0.5 mm) | `0.0005 m` |
| 3 | `0.0010 m` (1.0 mm) | `0.001 m` |
| 4 | `0.0001 m` (0.1 mm return) | `9.999999999999999e-05 m` |

All reported positions converged to their commands at the available count
resolution, including the reverse move from 1.0 mm to 0.1 mm. This demonstrates
repeatable bidirectional commanded/measured behavior over the tested first
millimetre. The operator confirmed that positive commands moved in the expected
physical direction and that there was no abnormal sound, alarm, noticeable
delay, or overshoot during the sequence. Task 2 is complete.

## Axis 1 Command Guard and Commissioning Limits

After Task 2, an accidental absolute command of `0.011 m` (11 mm) was
published through the unguarded forward controller. The drive generated alarm
`0xFF34` (Command pulse error) and stopped the command. After ROS exited, the
actual position was `10,000` counts, equal to 1.0 mm, so the actuator did not
travel to 11 mm. This demonstrated that the previous xacro `0..29 mm` metadata
was not sufficient runtime protection for direct topic commands.

Task 3 introduced a guarded public command path:

```text
/axis1_position_controller/commands
  -> azd3a_axis1_command_guard
  -> /axis1_raw_position_controller/commands
  -> ros2_control / EtherCAT
```

The guard:

- rejects non-finite, malformed, or out-of-range commands;
- permits only absolute targets from 0 to 5 mm;
- limits generated velocity to 0.5 mm/s;
- limits generated acceleration/deceleration to 1.0 mm/s^2;
- initializes its command from measured `/joint_states` feedback;
- publishes a smooth 200 Hz CSP position trajectory to the renamed raw
controller.

Both Axis 1 xacros now distinguish the manufacturer envelope (30 mm stroke,
40 mm/s maximum velocity, 40 N effort metadata) from the tighter runtime
commissioning settings. The public topic used by existing commands and the video
demo is unchanged, but it now passes through the runtime guard. The code,
xacros, and existing tests validated successfully, and `motor_controller` was
rebuilt.

The hardware rejection test then passed. With the drive in Operation Enabled,
the public topic reported `azd3a_axis1_command_guard` as its only subscriber.
Publishing the previous accidental target `0.011 m` produced:

```text
REJECTED Axis 1 target 0.011000 m: permitted range is 0.000..0.005 m
```

The drive remained Operation Enabled with no new fault, and the rejected
command was not forwarded to the raw controller. A valid bounded-motion test
then commanded an absolute `0.0005 m` target from the guarded public topic.
The guard logged `Accepted Axis 1 target 0.500 mm`, and settled feedback was:

```text
0.0004992999999999999 m = 0.4993 mm = 4,993 counts
```

The 7-count (`0.0007 mm`) difference from the 5,000-count target is inside the
actuator's ±0.01 mm repetitive-positioning specification. No new fault was
observed. This validates both rejection of unsafe commands and forwarding of a
valid velocity/acceleration-limited trajectory through the guarded path.

The official DR28T1A03-AZAKR catalog also specifies maximum acceleration of
`0.2 m/s^2` and minimum travel amount of `0.001 mm`. The command guard encodes
the manufacturer envelope as non-overridable caps:

```text
position:     0..30 mm
velocity:     at most 40 mm/s
acceleration: at most 0.2 m/s^2
increment:    multiples of 0.001 mm
```

After successful rejection and bounded-motion testing in the first 5 mm, the
Axis 1 motion launch commissioning range was expanded to `0..15 mm` for a
recorded mid-stroke round trip. Velocity remains `0.5 mm/s` and acceleration
remains `0.001 m/s^2`. The feedback-only configuration retains its 5 mm command
metadata because it is not intended to command motion. Invalid guard
configurations beyond the catalog
caps make the node fail rather than silently accepting an unsafe setting.
The catalog's 40 N force and 4 kg payload ratings are documented but cannot be
enforced by the current position-only PDO mapping because effort/load feedback
is not exported.

## Axis 1 Mid-stroke Recording

After the guarded rejection and 0.5 mm motion tests passed, the motion-launch
commissioning boundary was expanded from 5 mm to 15 mm. The manufacturer hard
caps remained unchanged. The operator recorded a guarded absolute move to
15 mm and a return to the selected safe near-home position of 1 mm using:

```bash
ros2 topic pub --once /axis1_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.015]}"

ros2 topic pub --once /axis1_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.001]}"
```

The guard settings were 0.5 mm/s maximum velocity and 1 mm/s^2 maximum
acceleration/deceleration. The operator reported the recording task finished.
The video is evidence of visible guarded Axis 1 motion; the exact settled
15 mm and return `/joint_states` messages were not provided in this session,
so measured endpoint convergence for this particular round trip is not claimed.
Earlier Task 1/2 moves do have captured measured convergence.

## Axis 1 Handoff to Axis 2

Axis 1 has now demonstrated:

- stable EtherCAT OP with DC Sync0 at a supported 5 ms cycle;
- exact measured feedback scaling at 10,000 counts/mm;
- repeatable bidirectional absolute positioning;
- rejection of an unsafe 11 mm command outside the then-active boundary;
- a guarded 0..15 mm commissioning range;
- velocity- and acceleration-limited CSP command generation;
- physical/video motion evidence with no reported abnormal behavior during the
  completed tests.

Axis 2 work may now begin at the read-only stage. Do not enable Axis 2 until
its `AZM46AK-FC7.2UA` identity, live position, electronic gear/resolution,
velocity-unit conversion, safe rpm limit, direction, and stop behavior have
been verified. Do not reuse Axis 3's FC20DA 20:1 reference scaling.

### Axis 2 read-only scaling preparation (2026-08-06)

The stationary Axis 2 baseline reported error `0x0000`, statusword `0x0270`,
mode 0, actual position 5,641,556 steps, and zero velocity. Further read-only
SDOs reported mechanism setting 1, gear-ratio override 0, rotation-direction
setting 1, and zero command/actual r/min.

Official Oriental Motor product data confirms `AZM46AK-FC7.2UA` is a 7.2:1 FC
geared motor with a permissible output speed of 0 to 416 r/min. The live
electronic gear A=1 and B=1 combines with the AZD3A manual formula
`resolution = 10,000 * B/A` to give 10,000 steps/output revolution. Therefore
Axis 2 position and `0x686C` velocity scaling are both
`2*pi/10,000 = 0.0006283185307179586` to produce ROS radians and radians/s.

An official AZ family catalog was added at
`vendor/oriental_motor/AZ_Family_Catalog_2018-2019.pdf`. A new staged launch,
`azd3a_axis2_feedback.launch.py`, uses only `GenericEcSlave`, Controlword=0,
mode=0, and state interfaces. It deliberately contains no Axis 2 command
interface and must be validated as feedback-only before any rotary motion.

The first live ROS feedback-only test succeeded. `/joint_states` reported
Axis 2 position `3544.694176883084 rad` and velocity `0.0 rad/s`. Dividing the
reported position by `2*pi/10,000` reproduces the raw absolute position of
5,641,556 steps exactly. `ros2 control list_hardware_interfaces` showed only
`axis2_joint/position` and `axis2_joint/velocity` under state interfaces, with
an empty command-interface section. Thus this launch cannot accept a motor
command through ros2_control, and the measured position/velocity conversion is
working as designed.

### Axis 2 first guarded rotation (2026-08-06)

The initial CSV launch exposed a limitation in the generic CiA 402 plugin: it
recognized only the standard Axis 1 object indices and therefore did not run
the state machine for Axis 2's objects at the standard indices plus `0x0800`.
The driver was extended with a configurable `cia402_object_index_offset` while
preserving zero as the single-axis/Axis 1 default. The Axis 2 xacro selects
`0x0800`. The complete driver suite passed 48 tests with zero failures.

After the fix, the live drive progressed through Switch On Disabled, Ready to
Switch On, Switch On, and Operation Enabled while holding zero speed. A guarded
5 rpm command then produced confirmed physical rotation. Viewed from the front
along the output-shaft axis, positive velocity was clockwise. The command
publisher ended after 6 seconds, the watchdog stopped the motor automatically,
motion was reported normal, and the final Axis 2 alarm remained `0x0000`.

The next commissioning boundary is 25 rpm with a 10 rpm/s acceleration and
deceleration ramp. This is one tenth of the 250 rpm requirement endpoint and
about 6 percent of the official 416 rpm motor limit; the guard still rejects
all commands above the active commissioning boundary.

The guarded operator interface was then changed to accept RPM directly on
`/axis2_velocity_controller/commands_rpm`. The raw hardware controller remains
in standard ROS rad/s; conversion is performed only inside the guard. This
makes an operator command such as `{data: [25.0]}` unambiguous while preserving
the standard ros2_control interface internally.

The operator later confirmed that the guarded 25 rpm physical test was
completed successfully. The final machine requirement remains 200-250 rpm;
25 rpm was a staged commissioning milestone rather than the final operating
speed. Further increases should remain staged and revalidate automatic stop,
feedback, vibration/noise, and the Axis 2 alarm after each level.

For the 200-250 rpm requirement, the Axis 2 launch now accepts explicit
`max_rpm`, `max_acceleration_rpm_s`, and `command_timeout_s` arguments. The
default ceiling remains the physically verified 25 rpm. The ros2_control raw
velocity boundary is 250 rpm (26.1799387799 rad/s), while the runtime guard
enforces the ceiling selected for each launch. The planned sequence is 100 rpm
as an intermediate check, followed by separate 200 rpm and 250 rpm runs, using
a conservative 25 rpm/s ramp and alarm/feedback validation after each run.

On 2026-08-14, the staged Axis 2 validation was completed successfully. The
operator physically confirmed smooth motion at 100 rpm, followed by 200 rpm
and the final 250 rpm requirement. After each timed command, the watchdog and
25 rpm/s guard ramp returned reported velocity to `0.0`; after the 200 rpm run,
Axis 2 error object `0x683F` read `0x0000`. The operator reported that the
250 rpm run also completed normally. Axis 2 guarded velocity control is
therefore verified across the required 200-250 rpm range in the tested
clockwise direction.

### Axis 1 retained-position startup fault and interlock (2026-08-06)

After the earlier recording, Axis 1 was stopped with `Ctrl+C` while physically
near 15 mm instead of being commanded back toward the beginning of the stroke.
That retained position was valid: the subsequent read was 149,993 counts, or
14.9993 mm. Stopping away from origin was not itself a drive error and should
remain a supported shutdown condition.

On the next Axis 1 CSP launch, the drive reached Operation Enabled and then
immediately faulted with statusword `0x0238`; SDO `0x603F` again reported
`0xFF34` (command pulse error). The failure mechanism was a startup handoff:
the generic position controller briefly supplied its default zero before the
Axis 1 guard initialized from `/joint_states`. From a retained 14.9993 mm
position, that created an approximately 15 mm one-cycle demand discontinuity.

The CiA 402 plugin now supports an optional `position_startup_tolerance` CSP
interlock. Axis 1 configures it as `0.00001 m` (0.01 mm). Until the raw
controller command agrees with measured feedback within that tolerance, the
plugin ignores the unmatched controller value and continuously holds the
measured position. Once the guard publishes the feedback-matched starting
value, normal guarded CSP commands are admitted. This removes any requirement
to return Axis 1 to origin before stopping ROS while retaining the existing
position, velocity, acceleration, and stroke guards.

The driver and repository suites passed 49 tests with zero failures after this
change, including a regression test that begins with measured position 15 and
controller command zero, verifies that measured position is held, and then
verifies synchronization when the command reaches 15.

The first live attempt showed one additional startup detail: `/joint_states`
can publish a temporary zero before EtherCAT reaches OP. The guard had latched
that first zero even though later feedback correctly reported 14.9993 mm. The
raw controller was active and received the requested 0.1 mm command, but the
CSP interlock correctly kept holding the measured 14.9993 mm because startup
had never synchronized. No movement or new alarm occurred during that blocked
attempt.

The guard was updated to follow measured feedback continuously until the first
accepted operator command. It therefore replaces temporary pre-OP feedback
with the true retained position, allowing the CSP interlock to synchronize
before motion. After rebuilding, Axis 1 relaunched from the retained ~15 mm
position, reached Operation Enabled without `0xFF34`, and physically moved
under the guarded return command. This live result validates startup from a
non-origin retained position; returning to zero before shutdown is not
required.

### Axis 3 feedback and 5 rpm commissioning (2026-08-14)

Axis 3 is the `AZM46AK-FC20DA` 20:1 geared rotary Motor 5. Its machine
requirement is approximately 20 rpm and its official permissible output speed
is 150 rpm. Live read-only state showed error `0x0000`, statusword `0x0270`,
mode 0, position -1,970,446 counts, zero velocity, electronic gear A=1/B=1,
and supported modes `0x1A5`.

The feedback-only ROS launch mapped Axis 3 RxPDO `0x1620` and TxPDO `0x1A21`
without a command interface. `/joint_states` reported -619.0338677895394 rad,
exactly matching the -1,970,446 raw counts at the 20,000-count/output-revolution
reference scaling, and velocity remained `0.0`.

A guarded CSV launch then used RxPDO `0x1622`, the Axis 3 CiA 402 object offset
`0x1000`, a 5 rpm ceiling, 2 rpm/s ramp, and 0.5-second watchdog. Physical
rotation was confirmed, provisionally clockwise viewed from the output-shaft
front. Stopping EtherCAT too soon initially produced `0xFF81`, which the vendor
manual identifies as Network bus error because the EtherCAT state left OP
during operation. After resetting Axis 3 and repeating while waiting for the
watchdog ramp to reach zero before `Ctrl+C`, the final alarm was `0x0000`.

The staged velocity test then passed at 10 rpm and at the final required
20 rpm. Axis 3 accelerated at 5 rpm/s, rotated smoothly with normal reported
behavior, returned to zero velocity through the watchdog ramp, and remained
alarm-free after communication was stopped only once physical and reported
velocity were zero. Axis 3 guarded velocity control is therefore verified at
its 20 rpm machine requirement in the tested positive/clockwise direction.

The operator then clarified the sequence requirement: Axis 3 is not merely a
continuous 20 rpm spin/stop axis. From its captured starting position it must
rotate 90 degrees clockwise and then rotate 90 degrees counterclockwise back to
that same starting position. With the verified 20,000-count/output-revolution
scaling, each 90-degree leg is exactly 5,000 counts or pi/2 radians. Positive
velocity was physically observed as clockwise, so the intended relative ROS
targets are `start + pi/2`, followed by `start`. Timed velocity commands are
not sufficient for this sequence because they cannot guarantee the final
angle; the next implementation must use guarded position trajectory generation
with a 20 rpm speed ceiling and feedback-verified completion of each leg.

#### Geared-output scaling correction (2026-08-14)

The first 90-degree CSP test changed feedback by the intended pi/2 radians but
produced only about 9 degrees of visible machine-output motion. This falsified
the earlier 20,000-count/output-revolution assumption. HM-60323-7E section 3-2
states that `10,000 * B/A` is resolution per revolution of the **motor output
shaft**. The external FC gear ratio must also be applied:

```text
Axis 2 FC7.2: 10,000 * 7.2 = 72,000 counts/machine-output revolution
Axis 3 FC20:  10,000 * 20  = 200,000 counts/machine-output revolution
Axis 3 90 degrees = 50,000 counts
```

Axis 3's observed 5,000-count move becoming about 9 output degrees matches the
FC20 calculation exactly. The corrected Axis 3 state factor is
`0.00003141592653589793 output rad/count`, and its command factor is
`31830.98861837907 counts/output rad`. The corresponding Axis 2 factors are
`0.00008726646259971647 output rad/count` and
`11459.155902616465 counts/output rad`.

This correction invalidates the claimed machine-output speeds from the earlier
Axis 2 and Axis 3 velocity milestones: those tests verified smooth motor-side
motion, direction, watchdog stopping, and alarm-free operation, but not the
required geared-output rpm. Axis 2 must be recommissioned toward 200-250 output
rpm and Axis 3 toward 20 output rpm using the corrected factors. No further
motion is permitted with the superseded scaling.

Axis 2 was subsequently recommissioned with the corrected FC7.2 conversion.
True machine-output tests passed at 25, 100, 200, and 250 rpm with smooth CW
rotation, controlled watchdog stopping, zero final velocity, and normal drive
state. The operator then increased output acceleration in stages and confirmed
that 100 rpm/s and finally 250 rpm/s worked on the present commissioning
hardware; 250 rpm/s reaches 250 rpm in approximately one second. This is an
empirical unloaded/current-mechanism result, not a manufacturer acceleration
rating. It must be revalidated against torque margin, load inertia, vibration,
and stopping behavior when the final Yuzu holding mechanism/load is installed.

Axis 2 now satisfies its corrected 200-250 machine-output rpm commissioning
requirement. The official `AZM46AK-FC7.2UA` permissible output-speed range is
0-416 rpm, so the required speed is within the product envelope.

After rebuilding with the FC20 correction, Axis 3 was relaunched with a 1 rpm
machine-output ceiling and commanded to the PDF's 90-degree CW positioning
target. Feedback changed from `-57.165142491030196` to
`-55.5943461642353 rad`, an exact `+1.570796326794896 rad` (90 degrees), while
the operator physically confirmed a quarter-turn at the output. Final velocity
was `0.0`. This validates 200,000 counts per FC20 machine-output revolution and
completes the Motor 5 peeler-positioning milestone. The separate PDF phase of
CW rotation at 20 output rpm remains pending corrected-speed commissioning.

Corrected-speed commissioning then passed at 5 and 10 machine-output rpm,
followed by the final 20 output rpm requirement. With the FC20 conversion,
full-speed feedback corresponds to approximately `2.0944 rad/s`. The operator
confirmed smooth clockwise rotation, controlled watchdog deceleration, zero
final velocity, and normal drive state. These separate tests validate the
corrected position and velocity scaling, but they do not yet validate an exact
90-degree index whose trajectory reaches 20 rpm. That combined test remains
pending. No automatic CCW return is included in the current PDF-following
behavior.

The source sequence artifacts were then added as `motion_cmd/sequence.png` and
`motion_cmd/YuzuSequence.pdf`. Review confirmed the final machine has six
logical motors, while the currently available AZD3A-KED exposes only three
local axes and is wired in a temporary commissioning order: local Axis 1 uses
a Motor 3/4 actuator type, local Axis 2 is machine Motor 2, and local Axis 3 is
machine Motor 5. A second three-axis AZD3A-KED will be added when the remaining
motors arrive. Final software must therefore keep logical machine motor IDs
separate from `{EtherCAT slave, local axis}` and load the wiring map from
configuration instead of renaming machine motors to match today's ports.

The PDF explicitly shows Motor 5 `90° ROT` during peeler positioning and later
CW rotation during rotation/feed. It does not explicitly show a 90-degree CCW
return; that return is recorded as a separate operator-confirmed requirement
until the source sequence drawing is revised.

## Daily startup reference

The single after-reboot and troubleshooting reference is now:

```text
motion_cmd/AZD3A_DAILY_STARTUP_TROUBLESHOOTING.md
```

It records the required EtherCAT master restart, `/dev/EtherCAT0` permissions,
NIC/slave checks, ROS environment sourcing, axis-specific alarm/status/position
reads, alarm reset sequence, known `0xFF34` diagnosis, and common recovery
commands. The EtherCAT master must be started after Ubuntu boots before any ROS
motor launch.
