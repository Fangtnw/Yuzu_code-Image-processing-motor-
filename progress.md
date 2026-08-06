# EtherCAT Bring-up Progress

Last updated: 2026-07-30

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
| Axis 3 | `AZM46AK-FC20DA` | Machine Motor 5 rotary axis; 20 rpm rotational target role |

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
