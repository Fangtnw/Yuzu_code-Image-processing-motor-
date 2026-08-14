# AZD3A Daily Startup and Troubleshooting

Use this file after reboot and whenever EtherCAT or an AZD3A axis does not
start normally. Commands assume Ubuntu 22.04, IgH EtherCAT at `/opt/etherlab`,
NIC `eno2`, and ROS 2 Humble workspace `~/kyutech/azd3a_ws`.

## Safety first

- Connect the dedicated Ethernet cable from `eno2` to AZD3A `ECAT IN`.
- Turn on the required AZD3A control/main power and confirm grounding.
- Keep the mechanism clear and the physical power cutoff accessible.
- Run only one AZD3A ROS launch at a time.
- Stop ROS with `Ctrl+C` before using SDO `upload` or `download` commands.

## After every Ubuntu reboot

Start or restart the EtherCAT master first:

```bash
sudo /etc/init.d/ethercat restart
```

Confirm the device, NIC, master, and slave:

```bash
ls -l /dev/EtherCAT0
ip link show eno2
/opt/etherlab/bin/ethercat master
/opt/etherlab/bin/ethercat slaves
```

Expected essentials:

```text
/dev/EtherCAT0: group ethercat, mode crw-rw----
eno2: UP, LOWER_UP
Master0: Link UP, Slaves: 1
0  0:0  PREOP  +  AZD3A-KED rev0301
```

Then source ROS in every new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/kyutech/azd3a_ws/install/setup.bash
```

## Known launch commands

Axis 1 guarded position control:

```bash
ros2 launch motor_controller azd3a_axis1_tiny_move.launch.py
```

Axis 2 feedback only, with no command interface:

```bash
ros2 launch motor_controller azd3a_axis2_feedback.launch.py
```

Axis 2 guarded RPM control:

```bash
ros2 launch motor_controller azd3a_axis2_tiny_spin.launch.py
```

The default remains the already verified 25 rpm commissioning ceiling. For
the final 200-250 rpm requirement, set the ceiling explicitly and validate one
level at a time. The raw ros2_control boundary is 250 rpm and the guard rejects
commands above the selected launch ceiling.

```bash
# Intermediate validation (recommended before the final range)
ros2 launch motor_controller azd3a_axis2_tiny_spin.launch.py \
  max_rpm:=100 max_acceleration_rpm_s:=25

# Requirement tests, only after the previous level passes
ros2 launch motor_controller azd3a_axis2_tiny_spin.launch.py \
  max_rpm:=200 max_acceleration_rpm_s:=25

ros2 launch motor_controller azd3a_axis2_tiny_spin.launch.py \
  max_rpm:=250 max_acceleration_rpm_s:=25
```

The current Axis 2 commissioning guard accepts RPM directly on:

```text
/axis2_velocity_controller/commands_rpm
```

Example command, after launching with a matching or higher ceiling:

```bash
timeout 16s ros2 topic pub --rate 10 \
  /axis2_velocity_controller/commands_rpm \
  std_msgs/msg/Float64MultiArray \
  "{data: [200.0]}"
```

When `timeout` stops the publisher, the 0.5-second watchdog commands zero and
the guard applies the configured deceleration ramp. With 25 rpm/s, allow 4,
8, and 10 seconds to stop from 100, 200, and 250 rpm respectively before
stopping the launch. Confirm `/joint_states` velocity is `0.0`, then stop ROS
and read Axis 2 alarm `0x683F`; expected value is `0x0000`.

Important correction (2026-08-14): the earlier RPM runs omitted the FC7.2
gearhead from the ROS conversion. They verified motion and stopping, not
200-250 rpm at the machine output. Axis 2 must be recommissioned with the
corrected 72,000-count/output-revolution scaling.

Final corrected status: true 25, 100, 200, and 250 machine-output rpm tests
passed. A 250 output-rpm/s acceleration also passed on the current test
hardware, but it is an empirical commissioning value and must be revalidated
with the final load installed.

Axis 3 guarded RPM control:

```bash
ros2 launch motor_controller azd3a_axis3_tiny_spin.launch.py \
  max_rpm:=20 max_acceleration_rpm_s:=5
```

Publish the verified 20 rpm requirement with:

```bash
timeout 10s ros2 topic pub --rate 10 \
  /axis3_velocity_controller/commands_rpm \
  std_msgs/msg/Float64MultiArray \
  "{data: [20.0]}"
```

Important correction (2026-08-14): physical 90-degree indexing exposed that
the FC20 gearhead was omitted. Axis 3 uses 200,000 counts per machine-output
revolution; the earlier nominal 20 rpm run was about 1 output rpm. Positive
velocity appeared clockwise from the output-shaft front. Wait until
physical motion stops and `/joint_states` velocity is `0.0` before stopping
the launch; otherwise the drive can report network-bus alarm `0xFF81`.

Final corrected status: Axis 3 uses 200,000 counts per FC20 output revolution.
Its physical 90-degree CW index and staged 5, 10, and 20 machine-output rpm CW
tests passed. At 20 rpm, expected ROS velocity is approximately
`2.0944 rad/s`.

## Read alarm, status, and position

Stop the ROS launch first. Axis-specific CiA 402 objects are:

```bash
# Axis 1
/opt/etherlab/bin/ethercat upload -p 0 --type uint16 0x603F 0
/opt/etherlab/bin/ethercat upload -p 0 --type uint16 0x6041 0
/opt/etherlab/bin/ethercat upload -p 0 --type int32  0x6064 0

# Axis 2
/opt/etherlab/bin/ethercat upload -p 0 --type uint16 0x683F 0
/opt/etherlab/bin/ethercat upload -p 0 --type uint16 0x6841 0
/opt/etherlab/bin/ethercat upload -p 0 --type int32  0x6864 0

# Axis 3
/opt/etherlab/bin/ethercat upload -p 0 --type uint16 0x703F 0
/opt/etherlab/bin/ethercat upload -p 0 --type uint16 0x7041 0
/opt/etherlab/bin/ethercat upload -p 0 --type int32  0x7064 0
```

An error code of `0x0000` means no drive alarm. Statusword `0x0270` (`624`)
while ROS is stopped is the normal Switch On Disabled state.

## Reset an AZD3A axis alarm

Do not reset until the cause is understood and the mechanism is safe. Stop ROS
first. Toggle the axis-specific alarm-reset input at `0x40C0`:

```bash
# Set this to 1, 2, or 3 for the axis being reset.
azd_axis_to_reset=1
/opt/etherlab/bin/ethercat download -p 0 --type uint8 0x40C0 "$azd_axis_to_reset" 0
/opt/etherlab/bin/ethercat download -p 0 --type uint8 0x40C0 "$azd_axis_to_reset" 1
sleep 1
/opt/etherlab/bin/ethercat download -p 0 --type uint8 0x40C0 "$azd_axis_to_reset" 0
```

For example, Axis 1 uses subindex 1. Verify afterward:

```bash
/opt/etherlab/bin/ethercat upload -p 0 --type uint16 0x603F 0
```

## Known alarm: `0xFF34`

`0xFF34` is the AZD3A command-pulse error encountered during Axis 1 CSP tests.
It can be caused by an unrealistically large one-cycle target change. Before
resetting:

1. Stop ROS.
2. Read `0x603F`, `0x6041`, and `0x6064`.
3. Check that the requested position and retained physical position agree.
4. Reset only after the command source is stopped.

The current Axis 1 implementation includes a startup-position interlock. It
holds measured position until the guard and controller synchronize, so Axis 1
may safely restart from a valid retained nonzero position.

## Common failures

### `/dev/EtherCAT0` does not exist

```bash
sudo /etc/init.d/ethercat restart
ls -l /dev/EtherCAT0
```

### Permission denied opening `/dev/EtherCAT0`

Check membership and permissions:

```bash
groups
ls -l /dev/EtherCAT0
```

Expected group membership includes `ethercat`. The persistent udev rule is:

```bash
sudo groupadd -f ethercat
sudo usermod -aG ethercat "$USER"
echo 'KERNEL=="EtherCAT[0-9]*", GROUP="ethercat", MODE="0660"' | sudo tee /etc/udev/rules.d/99-EtherCAT.rules
sudo udevadm control --reload-rules
sudo /etc/init.d/ethercat restart
```

Log out and back in after `usermod`, then check `groups` again.

### `The slave selection matches 0 slaves`

```bash
ip link show eno2
/opt/etherlab/bin/ethercat master
/opt/etherlab/bin/ethercat slaves
```

Check AZD3A power, the cable to `ECAT IN`, `eno2` link state, and the master
service. Do not run an SDO command until `ethercat slaves` lists position 0.

### SDO reports `Input/output error`

A running `ros2_control_node` may own the EtherCAT master. Stop the ROS launch
with `Ctrl+C`, confirm the slave is visible, and retry the SDO command.

### ROS package or launch is not found

```bash
source /opt/ros/humble/setup.bash
source ~/kyutech/azd3a_ws/install/setup.bash
ros2 pkg prefix motor_controller
```

If necessary, rebuild:

```bash
cd ~/kyutech/azd3a_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select motor_controller
source install/setup.bash
```

## Safe end of a session

- Axis 2: stop publishing RPM and wait until `/joint_states` velocity returns
  to `0.0` before pressing `Ctrl+C`.
- Axis 3: the command watchdog ramps to zero automatically, but do not press
  `Ctrl+C` until physical motion has stopped and `/joint_states` velocity is
  `0.0`. Stopping EtherCAT while the axis is operating can raise network-bus
  alarm `0xFF81`.
- Axis 1: it may stop at any valid guarded position; returning to zero is not
  required because the startup-position interlock holds retained feedback.
- Stop the active launch with `Ctrl+C` before removing motor power or the
  EtherCAT cable.
