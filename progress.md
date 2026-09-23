# EtherCAT Bring-up Progress

Last updated: 2026-09-21

## Goal

Bring up one Oriental Motor AZD3A-KED EtherCAT drive on native Ubuntu 22.04
with ROS 2 Humble. First prove discovery and read-only communication; only then
prepare a minimal one-axis `ros2_control` experiment.

## Current Result

On 2026-09-21 the drive was rewired to match the machine requirement order.
Axis 1 now carries logical Motor 1, an `AZM46AK` parameterized for the
`EZSM3LD040AZAK` 12 mm-lead, 400 mm-stroke linear slide. Axis 2 remains logical
Motor 2 (`AZM46AK-FC7.2UA`), and Axis 3 is disconnected. The former Axis 1
Motor 3/4 actuator configuration is historical and must not be used with the
new wiring.

Motor 1 read-only state is healthy: error `0x0000`, status `0x0270`, position
1,700 counts, and electronic gear A=1/B=1. The later ROS feedback-only launch
reported exactly 1,924 counts = 2.3088 mm. This gives 833,333.333 counts/m and
1.2e-6 m/count. A positive 0.9996 mm test then faulted with overload `0xFF30`
while the carriage was at the lower physical end. Operating/base current were
already 100%, and the overload timer was 5 seconds. After reset, feedback was
1,999 counts = 2.3988 mm. The guard now blocks further increasing-count motion
and permits a first -0.0996 mm direction test within 1.3992..2.3988 mm.

The reversed direction test passed. Motor 1 first moved from the 1,999-count
capture toward a 1,916-count target and settled at 1,917 counts. A subsequent
staged target of 1,582 counts settled at 1,586 counts = 1.9032 mm. The operator
confirmed that decreasing counts move the installed carriage upward, away from
the lower stop. Total measured upward travel was 413 counts = 0.4956 mm, with
no reported fault. The prior `0xFF30` overload is therefore consistent with
increasing counts driving downward into the lower mechanical stop.

The final target in the guarded recovery window was 1,166 counts = 1.3992 mm.
Motor 1 settled at 1,172 counts = 1.4064 mm, only 0.0072 mm from target. Total
measured travel from the 1,999-count post-reset lower-end capture was 0.9924 mm
upward. The final Axis 1 error object was `0x0000`. This verifies the corrected
12 mm-lead scaling, safe upward direction, guarded ramp, and approximately
1 mm assembled-system travel. Machine homing and a larger operational travel
envelope remain separate commissioning tasks.

The 1,999-count lower-end capture is now used as a provisional software zero;
the drive home offset was not changed. ROS position is positive upward, so the
last raw feedback of 1,172 counts becomes 0.9924 mm. The next guarded envelope
is 0..5 mm upward. Commissioning velocity was conservatively increased from
0.5 to 2 mm/s and acceleration from 1 to 5 mm/s^2. A true repeatable machine
home and the final requirement-based travel/speed remain pending.

The 2 mm/s test reached 1.9920 mm and held that position for two minutes while
enabled without an alarm. Motor 1 was then commanded to the 0.0996 mm safe
park before shutdown. The stopped raw position was 1,898 counts = 0.1212 mm
above provisional zero, and error `0x603F` remained `0x0000`. The guarded
minimum is now 0.0996 mm so an operator cannot command the captured lower
mechanical-end coordinate directly.

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
| Axis 1 | `AZM46AK` / `EZSM3LD040AZAK` | Machine Motor 1; 12 mm-lead, 400 mm-stroke linear slide, lower-end recovery window 1.3992-2.3988 mm |
| Axis 2 | `AZM46AK-FC7.2UA` | Machine Motor 2 rotary axis; 200-250 rpm spin/stop target role |
| Axis 3 | Disconnected | Previous Motor 5 commissioning connection removed during requirement-order rewiring |

The `DR28T1A03-AZAKR` is not throwaway hardware; it is the machine Motor 3/4
actuator type from the mechanism plan. For the current one-drive bring-up, one
of these actuators is wired to AZD3A Axis 1, so the software names it
`axis1_joint`. It is not the final machine Motor 1. It is a guided table
actuator with a 1 mm ball-screw lead, 30 mm stroke, 40 mm/s maximum speed, and
0.001 mm minimum travel amount. Axis 2 and Axis 3 are different rotary
motor/gearbox models, so their scaling must be confirmed separately from
physical labels, MEXE02, and live SDO settings before motion. Complete hardware
and preliminary scaling notes are recorded in `motion_cmd/AZD3A_HARDWARE.md`.

On 2026-09-21, logical Motor 4 was connected to AZD3A slave 1, local Axis 1
(CN7). The confirmed `DR28T1A03-AZAKR` has A=1/B=1 gearing and therefore uses
10,000 counts/mm. A guarded startup-relative +0.100 mm test moved from raw -37
to approximately raw 960, then returned to raw -34 with alarm `0x0000`. This
verifies EtherCAT addressing, scaling, forward motion, return motion, and
feedback for Motor 4. The next commissioning bound is +/-0.500 mm from the
startup feedback position at 0.5 mm/s.

The +0.500 mm follow-up reached exactly 4,966 counts (`0.0004966 m`) with no
alarm. After the launch was restarted at approximately 4,962 counts, the
guarded -0.500 mm recovery returned exactly to raw -34 (`-0.0000034 m`), again
with alarm `0x0000`. While diagnosing transient aggregate AL-state messages,
both slaves were confirmed OP and the Motor 4 domain reported WorkingCounter
3/3 with zero master frame loss. Motor 4 commissioning through +/-0.500 mm is
therefore complete; larger travel and machine homing remain pending.

The initial Motor 4-only two-slave launch left upstream slave 0 without cyclic
PDO output. Live sampling proved slave 0 oscillated through OP/INIT/SAFEOP while
slave 1 remained OP; AL status code `0x001B` identified a SyncManager watchdog.
The launch now includes slave 0 as a passive `GenericEcSlave` keepalive with
zero controlword/mode defaults, while Motor 4 remains the only enabled motion
axis. Motor 4's commissioning velocity was raised conservatively from 0.5 to
2.0 mm/s (catalog maximum 40 mm/s). Runtime validation of the keepalive change
is pending. Motor 3 remains disconnected and cannot be speed-tested yet.

Motor 4 was then integrated into the existing Motor 1+2 operator GUI and
two-slave backend. GUI commands are absolute ABZO positions from 0..15 mm in
0.001 mm increments. The Motor 4 guard selects 2 mm/s for moves through 1 mm,
5 mm/s through 5 mm, and 10 mm/s for longer moves, with corresponding guarded
accelerations of 0.020, 0.050, and 0.100 m/s^2. The internal command interface
allows only a -0.01 mm startup margin for the retained -0.0034 mm encoder
origin; public commands remain non-negative. Software validation passed, but
the combined three-motor runtime test was pending at that stage.

The first combined run showed Motor 4 feedback and a fully connected guarded
topic path, but accepted commands did not change drive object `0x607A`. The raw
controller topic correctly published `0.002 m` while `0x607A` stayed equal to
feedback at 4,965 counts. Source tracing identified the CSP startup
synchronizer: its one-count tolerance had not armed during asynchronous
multi-controller startup, so it intentionally overrode commands with measured
position. The combined Motor 4 startup tolerance is now one actuator command
increment (0.001 mm); larger retained-command jumps remain blocked. Runtime
retest is pending.

The one-increment tolerance retest still remained latched even though slave 1
reached Operation Enabled and the raw guarded topic continuously published the
requested `0.002 m`. The Motor 4 finite plugin latch was therefore removed.
Startup synchronization remains enforced by the Motor 4 application guard,
which continuously follows feedback before the first operator command and
rejects any command until valid feedback exists. Motor 1 retains its existing
separate startup tolerance. Runtime retest was pending at that stage.

The final configuration removed Motor 4's finite plugin latch and retained the
feedback-aware synchronization in `azd3a_motor4_position_guard.py`. After the
rebuild and restart, the operator confirmed physical Motor 4 movement from the
combined GUI while Motors 1 and 2 remained controllable. The combined two-slave
backend is therefore validated for step-by-step control of Motors 1, 2, and 4.
The domain had previously been confirmed at WorkingCounter 6/6 with both slaves
OP. Motor 4 now has a guarded absolute range of 0..15 mm with distance-adaptive
2/5/10 mm/s profiles. Motor 3 remains disconnected and pending commissioning.

The complete problem/fix record is in
`motion_cmd/MOTOR4_COMMISSIONING_POSTMORTEM.md`.

Motor 6 commissioning preparation began on AZD3A slave 1, local Axis 3. The
physical labels identify an Oriental Motor `AZM46AK-PS50` driving a MISUMI
`SVKA-150-795-25-...` conveyor. After connecting the motor and power-cycling
controller #2, Axis 3 reported alarm/error `0x0000`, status `0x0270`, and raw
position 2,280,574 counts. A=1/B=1 and the 50:1 gearbox give 500,000 counts per
output revolution. MISUMI specifies a 30 mm drive pulley, so 1 output rpm is
approximately 1.5708 mm/s belt speed. A separate watchdog-protected 1 rpm
commissioning launch is built; physical direction testing is pending.

Motor 6's first physical commissioning run is now complete. A guarded
`+1.0 rpm` command moved the conveyor smoothly in the forward direction. Raw
position changed from 2,280,574 to 2,358,024 counts: 77,450 counts, or 0.1549
output revolution and approximately 14.6 mm of travel on the 30 mm drive
pulley. After the timed command and watchdog stop, `/joint_states` reported
zero velocity and the Axis 3 alarm remained `0x0000`. This validates positive
direction, live scaling, controlled stopping, and the standalone Motor 6
commissioning path. The 1 rpm safety cap remains in place pending deliberate
higher-speed testing and integration into the combined operator GUI.

Motor 6 was subsequently integrated into the combined Motors 1/2/4 operator
GUI and two-slave backend. AZD3A #2 now has one composite slave mapping: Axis 1
Motor 4 CSP position plus Axis 3 Motor 6 CSV velocity. This prevents competing
ROS hardware instances from claiming the same EtherCAT slave. The guarded
Motor 6 ceiling was increased conservatively from 1 to 5 output rpm with a
2 rpm/s ramp and the existing 0.5-second watchdog. At the 5 rpm cap, calculated
belt speed is approximately 7.85 mm/s. Runtime validation of combined GUI
operation at the new limit is pending.

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

## Initial Yuzu operator GUI

This subsection records the initial Motors 1/2 implementation. It is retained
as history; the current Motors 1/2/4/6 state is recorded in the 2026-09-22
session closeout below and in `motion_cmd/YUZU_OPERATOR_GUI.md`.

A Tkinter/ROS 2 operator panel is available as `yuzu_operator_gui`. It provides
Motor 1 position and safe-park controls, Motor 2 guarded CW/CCW RPM and stop
controls, live feedback, backend availability, count alignment, and stale-data
indication. It publishes only to the existing guarded public topics. The GUI
limits Motor 1 to 0.0996..5 mm and Motor 2 to the validated 250 rpm requirement.

The EtherCAT CiA 402 plugin now supports a secondary state machine inside the
same physical multi-axis slave. The combined PDO configuration maps Axis 1 CSP
and Axis 2 CSV through one AZD3A slave instance, with separate ros2_control
position and velocity controllers and both existing guards. The launch
`azd3a_motor1_motor2_gui.launch.py` starts the combined backend and GUI, so the
sequence can command Motor 1 and Motor 2 step by step without restarting or
competing for EtherCAT ownership. GUI usage is documented in
`motion_cmd/YUZU_OPERATOR_GUI.md`. Software build, xacro expansion, launch
loading, 17 repository tests, and the 50-test driver suite passed at that
stage; live combined hardware validation was the next step at that time.

The single after-reboot and troubleshooting reference is now:

```text
motion_cmd/AZD3A_DAILY_STARTUP_TROUBLESHOOTING.md
```

It records the required EtherCAT master restart, `/dev/EtherCAT0` permissions,
NIC/slave checks, ROS environment sourcing, axis-specific alarm/status/position
reads, alarm reset sequence, known `0xFF34` diagnosis, and common recovery
commands. The EtherCAT master must be started after Ubuntu boots before any ROS
motor launch.

## Session closeout: Motor 6 conveyor and combined GUI (2026-09-22)

Motor 6 is confirmed as Oriental Motor `AZM46AK-PS50`, connected to AZD3A #2
(EtherCAT slave 1), local Axis 3, and driving the MISUMI
`SVKA-150-795-25-NV-NM-NH-W-R-...` conveyor. The researched motor/conveyor
specifications, provenance, scaling calculations, and physical measurements
are recorded in `motion_cmd/AZD3A_HARDWARE.md`. The official MISUMI catalog is
archived at `vendor/misumi/SVKA_catalog_2019_pages_1254-1255.pdf`; its source,
retrieval date, and SHA-256 are recorded in `vendor/misumi/README.md`.

The first guarded Motor 6 test at +1 output rpm moved the conveyor smoothly in
the operator-confirmed forward direction. Raw position increased from
2,280,574 to 2,358,024 counts (77,450 counts, 0.1549 output revolution, about
14.6 mm belt travel). The timed command stopped normally, feedback velocity
returned to zero, and Axis 3 alarm remained `0x0000`.

Motor 6 is now included in the combined Motors 1/2/4/6 operator GUI. AZD3A #2
uses one composite mapping for Motor 4 Axis 1 CSP and Motor 6 Axis 3 CSV, so
only one hardware instance owns the slave. The GUI provides forward/reverse,
start/stop, live RPM and calculated belt speed, plus a global Motor 2/Motor 6
stop. Motor 6 is guarded to 5 output rpm, ramps at 2 rpm/s, and retains the
0.5-second watchdog. At 5 rpm the calculated belt speed is about 7.85 mm/s.

Software closeout passed: 21 repository tests, Python syntax checks, combined
xacro/XML expansion, `motor_controller` package build, and installed launch
description loading. The next session should physically validate the combined
Motor 6 GUI path at 2 rpm first, then 5 rpm if motion remains smooth. Motors 1,
2, and 4 were already physically validated through the combined GUI. Motor 3
remains disconnected.

## Updated linear motion requirements and GUI details (2026-09-22)

The detailed requirements now set Motor 1 velocity to 15 mm/s and Motors 3/4
velocity to 8 mm/s. Motor 1's guarded travel was expanded from 5 mm to 200 mm,
which is half of the `EZSM3LD040AZAK` 400 mm catalog stroke. Its lower bound
remains 0.0996 mm above the provisional lower-end zero. Because no new
acceleration/deceleration values were supplied, Motor 1 retains a symmetric
5 mm/s^2 software ramp.

Motor 4 now uses a fixed 8 mm/s profile over its existing guarded 0..15 mm
range, with the retained symmetric 50 mm/s^2 ramp. Motor 3's 8 mm/s requirement
is recorded but its controls remain disabled because Motor 3 is physically
disconnected and its scaling, travel, direction, acceleration, and deceleration
are not yet commissioned.

The operator GUI now includes a collapsible read-only **Motion details** table
showing range, velocity, acceleration, and deceleration for Motors 1, 2, 3, 4,
and 6. This exposes the actual guard values without allowing operators to
bypass them. Software validation after this update passed 21 repository tests,
Python syntax checks, combined xacro/XML expansion, and the ROS workspace
package build.
Physical validation should begin with short Motor 1 and Motor 4 moves before
using the expanded travel.

### Updated guarded speed and ramp profiles (2026-09-22)

The operator-requested ramps are now applied consistently in the guards,
combined launch, GUI motion-details panel, and regression tests:

- Motor 1 remains capped at 15 mm/s over 0.0996..200 mm, with symmetric
  15 mm/s^2 acceleration/deceleration.
- Motor 5 is capped at 20 rpm within its runtime-zeroed +/-90 degree range,
  with symmetric 20 rpm/s acceleration/deceleration.
- Motor 6 remains capped at 50 output rpm (78.54 mm/s on the verified 30 mm
  pulley), with a symmetric 25 rpm/s acceleration/deceleration ramp and the
  0.5-second watchdog.

These are software limits, not a substitute for a staged physical test after
restart. Begin at small Motor 5 angles and low conveyor belt speed, verify
feedback and alarm `0x0000`, then increase toward the new ceilings.

### Operator GUI defaults and step controls (2026-09-23)

The GUI arrow controls now sit beside each motor's step-size field and publish
the guarded command immediately. Motor 1 exposes the full 0..400 mm range,
uses Return to 0 mm, and defaults to a 1 mm step. Motors 3 and 4 default to
1 mm steps. Motor 5 starts at 0 degrees with a 90-degree step while retaining
the runtime-zero interlock. Motor 2 defaults to 200 rpm and Motor 6 defaults
to 50 mm/s.

Motor 6 remains limited to the verified 50 output-rpm backend ceiling
(78.54 mm/s with the 30 mm pulley). A 100 mm/s GUI ceiling would require about
63.7 rpm and therefore exceeds the documented 60 rpm drive limit; this must be
resolved by confirming a higher-rated drive/pulley before raising the guard.

Motor 5's step arrows now provide a separate guarded manual-jog topic before
zero capture. The operator can jog in 90-degree increments, wait for the
mechanism to stop, and then capture the current position as zero; typed angle
and Return-to-origin commands remain unavailable until that zero is captured.

### Hardware step sizes and rotary profiles (2026-09-23)

The GUI now labels hardware-derived step sizes rather than calling them
generic sensitivity: Motor 1 is 0.0012 mm/count, Motors 3 and 4 are
0.0001 mm/count, and Motor 5 is 0.0018 degrees/count (200,000 counts/output
revolution). Motor 2's 250 rpm value is the current project operating limit;
the AZD3A drive envelope is 416 rpm.

Motor 2 and Motor 6 GUI start commands now include operator-entered
acceleration and deceleration in rpm/s. Each guard validates these values
against its configured maximum before applying the ramp, while the watchdog
and existing speed limits remain active.

### Hardware envelope view and expanded linear limits (2026-09-23)

Motors 3 and 4 are now guarded to 20 mm with 0.0001 mm hardware-count steps;
the catalog stroke is 30 mm, so the 20 mm value remains a project envelope.
Motor 6 now uses a round 90 mm/s GUI limit (about 57.3 rpm), below the 60 rpm
drive maximum and its approximately 94.25 mm/s pulley speed. The GUI's
scrollable Motion details section now lists these hardware envelopes separately
from project limits.

Motor 2's 25 rpm/s acceleration/deceleration is a software commissioning cap,
not a manufacturer limit; 250 rpm/s was previously observed on unloaded test
hardware but requires load validation. Motor 5's 20 rpm/s profile is likewise a
software cap; its hardware data specifies 150 rpm output speed but no angular
travel limit. The GUI now permits +/-180 degrees after runtime zero; confirm
the mechanical fixture's safe travel before using the expanded range.

### Motor 2 ramp ceiling raised for speed matching (2026-09-23)

The combined GUI launch and standalone Axis 2 commissioning launch now permit
up to 500 rpm/s acceleration and deceleration for Motor 2, matching the
250 rpm operating speed more aggressively. The GUI defaults both fields to
500 rpm/s but still allows lower values. This is a software/commissioning
ceiling rather than a drive datasheet rating; validate 25, 100, 250, then
500 rpm/s under the actual Yuzu load and stop if the mechanism vibrates,
stalls, or alarms.

Motor 6 now has relative conveyor distance-step controls in the GUI. Each
step uses fresh encoder position feedback, the configured belt speed, and the
guarded velocity ramp, then requests a controlled stop when the requested
distance is reached. The existing continuous speed start/stop controls remain
available.

The default Motor 6 distance step is now 30 mm. A new Operation sequence tab
maps the peeling workflow into guarded, operator-advanced stages with editable
Motor 1 approach, Motor 3/4 feed and gripping distances, Motor 2 speed, and
Motor 6 positioning distance.
The operation defaults for Motor 1 are now 300 mm approach and 400 mm home;
the tab keeps every sequence step visible and highlights only the active one.
The GUI now maps each drive's CiA-402 status word and disables motion controls
until all six drives are Operation Enabled (`0x0567` pattern), preventing
commands during startup or after a fault.

The Motor 6 step controller now uses the configured deceleration to begin
braking at the calculated stopping distance. Its GUI ramp default/combined
guard ceiling is 250 rpm/s, reducing the braking distance for short 10 mm
steps compared with the former 25 rpm/s setting. Actual encoder travel is
shown on a separate line beneath measured belt speed.

### Motor 6 distance-step encoder-scale correction (2026-09-23)

The first combined-GUI distance-step test exposed an encoder scaling error:
the newly mapped Motor 6 position used `1e-7`, while the drive's verified
position/velocity scale is `0.000012566370614359173`. The incorrect scale
under-reported encoder travel by approximately 125.66x, causing a 10 mm step
to run substantially too far before the stop condition. The combined map now
uses the verified scale; retest with a short 10 mm step before longer moves.

### EtherCAT deployment recovery after kernel update (2026-09-23)

After an Ubuntu kernel update, the machine booted `6.8.0-138-generic` while
IgH 1.6.9 modules existed only for `6.8.0-124-generic`. The service therefore
reported `modprobe: FATAL: Module ec_master not found`. Rebuilding IgH against
the active kernel and installing `ec_master.ko`/`ec_generic.ko` restored module
loading. The next restart exposed a separate deployment configuration issue:
`/etc/sysconfig/ethercat` had no `MASTER0_DEVICE`, so the service reported
`No network cards for EtherCAT specified` and `/dev/EtherCAT0` was absent.

For customer deployment, install matching modules for the active kernel,
verify `modinfo ec_master` and `modinfo ec_generic`, configure
`MASTER0_DEVICE` to the customer's dedicated EtherCAT NIC (MAC or interface),
set `DEVICE_MODULES="generic"`, then restart and verify `ethercat slaves`.

## Motor 6 50 rpm request and Motor 3 reconnection (2026-09-22)

Motor 6's guarded maximum was increased from 5 to 50 gearbox-output rpm, still
below the `AZM46AK-PS50` catalog maximum of 60 rpm. For the conveyor operator,
linear belt speed is more meaningful than shaft rpm. The GUI now accepts mm/s
and converts internally using the verified 30 mm pulley: 50 rpm equals
78.5398 mm/s. The backend remains RPM-based because CSV scaling controls the
gearbox output. The existing symmetric 2 rpm/s ramp and 0.5-second watchdog are
retained; staged physical validation must begin at the GUI's 10 mm/s default.

Motor 3 is reported physically reconnected on AZD3A #1/slave 0 Axis 3. It is
not yet enabled in the GUI. Live alarm, status, position, electronic gearing,
and supported-mode reads are required first. Source inspection also confirmed
that the current composite CiA-402 plugin supports only a primary and one
secondary state machine per slave; slave 0 already uses those for Motors 1 and
2. A third-axis extension is therefore required to add Motor 3 without
competing for the physical slave or abusing an unrelated ROS command
interface.

The Motor 3 live read-only check then passed: Axis alarm and CiA-402 error were
`0x0000`, stopped status was `0x0270`, raw position was 2,726 counts,
electronic gearing was A=1/B=1, supported modes were `0x01A5`, and the
controller alarm bitmap was zero. A standalone guarded commissioning launch
was added for slave 0 Axis 3 CSP with the confirmed 10,000 counts/mm scaling.
It holds slave 1 in a passive non-enabled keepalive and limits initial Motor 3
motion to startup-relative +/-0.500 mm at 2 mm/s. Software validation passed
22 repository tests, Python syntax, xacro/XML expansion, and ROS package build.

The first Motor 3 guarded motion test commanded a startup-relative +0.100 mm.
Raw position changed from 2,726 to 3,721 counts, a measured increase of 995
counts or 0.0995 mm at 10,000 counts/mm. `/joint_states` settled at
0.0003721 m and Axis 3 alarm remained `0x0000`. This validates the address,
positive count scaling, and commanded/measured convergence for the first tiny
move. Physical direction, smoothness, and return-to-origin remain to be
confirmed before raising speed to the 8 mm/s requirement or enabling Motor 3
in the combined GUI.

The startup-relative 0.000 mm return then settled at raw 2,730 counts
(`0.0002730 m`), only four counts or 0.0004 mm from the captured 2,726-count
origin. Alarm remained `0x0000`. This confirms repeatable bidirectional
command/feedback behavior over the first 0.100 mm. Operator confirmation of
physical direction and smoothness is still required before the 0.500 mm and
8 mm/s stages.

The operator then confirmed that positive Motor 3 motion is the required
forward direction and that both directions were smooth with no unusual sound.
Direction, tiny-motion scaling, alarm-free stopping, and first-round-trip
repeatability are therefore validated. The next commissioning stage is a
+0.500 mm round trip at the current 2 mm/s guard speed.

The +0.500 mm stage then passed: feedback reached 0.0007729 m, corresponding
to 4,999 counts (0.4999 mm) above the approximate 2,730-count start. The zero
return settled at 0.0002732 m / 2,732 counts, about two counts (0.0002 mm) from
start, and alarm remained `0x0000`. The standalone next-stage guard is expanded
to +/-2.000 mm at the required 8 mm/s with a symmetric 50 mm/s^2 ramp; a 2 mm
move is long enough to reach the requested velocity before deceleration.

The Motor 3 requirement-speed round trip then passed. The +2.000 mm command
settled at 0.0022729 m, which is 19,999 counts / 1.9999 mm above the captured
approximately 2,730-count origin. The zero return settled at 0.0002732 m and
raw 2,732 counts, within about two counts / 0.0002 mm of start. Alarm remained
`0x0000`. This validates commanded/measured convergence and return
repeatability using the configured 8 mm/s, 50 mm/s^2 trajectory; operator
confirmation of physical smoothness at the higher speed remains pending.

The operator confirmed the 2 mm forward-and-return movement was physically
smooth at the configured 8 mm/s. Motor 3 standalone commissioning is complete:
connection, alarm state, A/B gearing, 10,000 counts/mm scaling, positive
direction, bidirectional repeatability, 8 mm/s speed, 50 mm/s^2 ramps, and
alarm-free stopping are validated. Motor 3 is ready for the third-state-machine
driver extension and combined-GUI integration.

## Motor 3 combined GUI integration (2026-09-22)

Motor 3 is now integrated into the normal two-controller GUI launch. The
EtherCAT CiA-402 plugin was extended with a third independent state machine for
slave 0 Axis 3 (object offset `0x1000`, CSP mode 8). The combined PDO map adds
`0x1620`/`0x1A20`, and ros2_control exposes a separate custom
`motor3_position` command/state interface without duplicating the physical
slave. A forward-command controller and feedback-synchronized absolute guard
provide a public 0..15 mm interface with 0.001 mm increments, 8 mm/s velocity,
and symmetric 50 mm/s^2 ramps.

The GUI Motor 3 panel is unlocked and reads its feedback from
`/dynamic_joint_states`. Its Move/Return buttons remain disabled unless the
guarded backend is subscribed. The guard follows measured Motor 3 feedback
until the first operator command, preventing a stale startup target.

Validation completed without issuing hardware motion: both modified ROS
packages compiled; the combined xacro and launch arguments loaded; all 23
repository tests passed; and the EtherCAT driver reported 51 test results with
zero errors/failures (including a new tertiary-axis state/mode/position-hold
regression test). Combined hardware validation is still pending and must start
by confirming live `motor3_position` feedback and `0x707A == 0x7064`, followed
by only a +0.100 mm absolute move from the observed starting position.

The first combined-GUI Motor 3 motion test passed. Starting feedback was
0.0002732 m / 2,732 counts, with target and actual differing by one count and
alarm `0x0000`. A GUI target of 0.373 mm settled at 0.0003727 m / 3,727 counts
against a 3,730-count target, a following difference of three counts
(0.0003 mm), with alarm still `0x0000`. The GUI Return to 0 mm action then
settled at exactly 0.0000000 m / 0 counts with alarm `0x0000`. This validates
the combined custom interface, GUI command path, absolute-zero return, and
command/feedback convergence. The operator confirmed that the physical motion
was smooth and the positive move travelled in the required forward direction.
Combined Motor 3 GUI commissioning is therefore complete.

## Motor 5 rewiring and isolated commissioning path (2026-09-22)

Motor 5, the previously tested `AZM46AK-FC20DA`, is now wired to AZD3A #2
(slave 1) Axis 2. Its read-only baseline passed with both slaves OP, Axis 2
alarm `0x0000`, CiA-402 error `0x0000`, stopped status `0x0270`, raw absolute
position -529,442 counts, A/B gearing 1:1, and supported modes `0x01A5`.

An isolated commissioning launch now maps Axis 2 PDOs `0x1610`/`0x1A11` using
the validated 200,000 counts/output-revolution FC20DA scaling. Slave 0 receives
a passive, non-enabling cyclic keepalive. The Motor 5 guard captures live
position as its origin and permits only a startup-relative 0..1 degree command
at 1 rpm with a 5 rpm/s ramp, preventing any jump toward the multi-turn ABZO
absolute zero. Syntax, xacro, launch loading, package build, and all 24 project
tests passed. Physical +1 degree and return tests remain pending.

The first isolated Motor 5 round trip passed electronically. The guarded +1
degree command moved from -529,442 to -528,886 counts, a +556-count change
equal to approximately +1.0008 degrees. Target and actual matched, final
velocity was zero, and alarm remained `0x0000`. The startup-relative zero
return settled at -529,441 counts, one count / approximately 0.0018 degrees
from the captured origin, again with zero final velocity and no alarm. Physical
motion was smooth in both directions and that positive rotation is the required
Motor 5 positioning direction. The next commissioning guard is expanded to
startup-relative +/-10 degrees at 5 rpm with a 5 rpm/s ramp. The eventual GUI
will expose explicit CW/CCW selection and publish a signed angle, allowing the
operator to reverse direction without changing wiring.

The Motor 5 +10-degree stage then passed. Target and actual both settled at
-523,882 counts, approximately +10.006 degrees from the prior origin, with
zero final velocity and alarm `0x0000`. Return settled at -529,437 counts,
four counts / approximately 0.0072 degrees from origin, again alarm-free. The
operator confirmed smooth physical movement.

Motor 5 is now software-integrated into the combined all-six-motor launch.
Slave 1 Axis 2 uses PDOs `0x1610`/`0x1A11` and the custom `motor5_position`
interface alongside Motor 4 (Axis 1) and Motor 6 (Axis 3). The GUI requires the
operator to press **Set current position as zero** after every launch before
angle controls unlock. It then accepts 0..90 degrees with explicit CW (+) and
CCW (-) selection and provides Return to Origin. The guard rejects motion
before zero capture and uses the validated 5 rpm, 5 rpm/s profile. All 25
project tests, Python syntax, combined xacro expansion, package build, and
launch loading passed. Combined startup and 90-degree physical validation are
pending.

The first combined startup check exposed a software-only interface ownership
error before any Motor 5 motion: `motor5_position` appeared under the slave 0
composite joint, leaving `motor5_raw_position_controller` inactive. Slave 1
Axis 2 remained non-enabled (mode/output bytes zero), its alarm remained
`0x0000`, and actual position stayed near -529,435 counts, so no unintended
motion occurred. The xacro was corrected so `motor5_position` and the tertiary
offset `0x0800` belong to slave 1's `motor4_joint` module. A structural
regression assertion now verifies this ownership. All 25 tests, xacro
expansion, and the package rebuild pass after the correction.

The next corrected combined startup showed Motor 5's controller active but
reported custom dynamic feedback as `0.0` while SDO actual position was
`-529435`. No movement occurred: Axis 2 target remained zero, alarm was
`0x0000`, and the drive stayed at its prior position. Inspection found the
combined `0x1A11` mapping omitted Axis 2's mapped velocity entry `0x686C`,
despite the domain reporting the full 11-byte PDO. That entry is now included
as an unexported state channel. The launch must be restarted and dynamic
feedback must match `0x6864` before the GUI runtime-zero button is used.

After a clean restart, temporary driver instrumentation confirmed both
tertiary channels: Motor 3 stored `3e-07 m` at state index 2 and Motor 5
stored `-16.6327 rad` at state index 2, exactly matching their TPDO values.
The instrumentation was removed and the driver rebuilt. The GUI now includes
`<`/`>` target nudges for Motors 1, 3, 4, and 5; these change the numeric field
only, while the existing Move button remains required to publish a command.
Motor 5's nudge is 1 degree and remains gated behind runtime-zero capture.

The operator clarified that `<` and `>` must command motion immediately. The
GUI implementation now provides per-motor step-size fields and publishes a
guarded command on each arrow press; typed targets, Move, and Return controls
remain available. Motors 1/3/4 step in millimetres and Motor 5 steps in
degrees, with Motor 5 still requiring the manually established runtime origin.
All 25 project tests and the GUI package rebuild pass.

Final combined all-six feedback check passed after the clean restart. Dynamic
state reported Motor 3 at `0.0005126 m`, Motor 4 at `0.0005128 m`, and Motor 5
at `-16.457987 rad` with live velocity feedback `0.0012315 rad/s`. This
confirms the corrected custom Motor 5 state path is live alongside Motors 1,
3, 4, and 6. Further GUI polish is intentionally deferred to the next session.
