# AZD3A-KED One-Axis Basic ROS2 Test

Use this for the first ROS2 Humble bring-up on native Ubuntu 22.04 with one
AZD3A-KED connected to the dedicated EtherCAT NIC.

## Safety Order

1. Confirm the motor moves correctly in MEXE02.
2. Connect only one AZD3A-KED to the Ubuntu EtherCAT NIC.
3. Keep the motor unloaded or mechanically safe.
4. Confirm 24/48 V main power, 24 V control power, and grounding.
5. Run status/discovery before publishing motion.
6. Start with tiny moves and low speed.
7. Keep a physical power cutoff ready.

## Build

From the ROS2 workspace that contains `motion_cmd`:

```bash
colcon build --packages-select motor_controller
source install/setup.bash
```

## EtherCAT Status Check

Run this before motion:

```bash
ros2 run motor_controller azd3a_ethercat_check
```

To see what it checks without touching hardware:

```bash
ros2 run motor_controller azd3a_ethercat_check --dry-run
```

If this fails, fix the IgH master service, NIC assignment, permissions, cable,
or AZD3A power before continuing. The `azd3a_basic_test` command below is only
for publishing simple motor commands after the EtherCAT layer is healthy.

## Command Dry Run

```bash
ros2 run motor_controller azd3a_basic_test --dry-run move-mm 1.0 0.5 --accel-mm-s2 1.0 --decel-mm-s2 1.0 --max-position-mm 5.0
```

Only remove `--dry-run` when a real AZD3A driver node is subscribed to the topic.

## Basic Commands

Linear position:

```bash
ros2 run motor_controller azd3a_basic_test move-mm 1.0 0.5 --accel-mm-s2 1.0 --decel-mm-s2 1.0 --max-position-mm 5.0
```

Rotary speed:

```bash
ros2 run motor_controller azd3a_basic_test spin-rpm 10.0 --accel-rpm-s 5.0 --decel-rpm-s 5.0 --max-rpm 30.0
```

Rotary angle:

```bash
ros2 run motor_controller azd3a_basic_test move-angle 15.0 5.0 --accel-rpm-s 3.0 --decel-rpm-s 3.0 --min-angle-deg -30.0 --max-angle-deg 30.0
```

Home:

```bash
ros2 run motor_controller azd3a_basic_test home
```

Stop:

```bash
ros2 run motor_controller azd3a_basic_test stop
```

Alarm reset topic, for a driver node that supports it:

```bash
ros2 run motor_controller azd3a_basic_test reset-alarm
```

## Topic Payloads

The script uses the simple existing test topics:

- `motor_cmd`: `std_msgs/Float32MultiArray`
  - `[position_mm, speed_mm_s, accel_mm_s2, decel_mm_s2]`
- `motor_spin`: `std_msgs/Float32MultiArray`
  - `[rpm, accel_rpm_s, decel_rpm_s]`
- `motor_angle_cmd`: `std_msgs/Float32MultiArray`
  - `[angle_deg, speed_rpm, accel_rpm_s, decel_rpm_s]`
- `motor_home`: `std_msgs/Empty`
- `motor_stop`: `std_msgs/Empty`
- `motor_reset_alarm`: `std_msgs/Empty`

Use `--topic-prefix /axis1` if your node is namespaced:

```bash
ros2 run motor_controller azd3a_basic_test --topic-prefix /axis1 move-mm 1.0 0.5 --max-position-mm 5.0
```

## First Hardware Success Condition

Stop after proving this:

1. `ethercat slaves` sees one AZD3A-KED.
2. Status can be read without alarm.
3. The motor accepts enable/ready state from the real driver.
4. One tiny position or angle command moves correctly.
5. Stop command works.

Do not add the full six-axis chain until this one-axis result is repeatable.
