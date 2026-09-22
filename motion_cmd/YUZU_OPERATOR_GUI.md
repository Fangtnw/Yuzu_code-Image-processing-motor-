# Yuzu Peeler Operator GUI

The operator panel controls guarded public interfaces for all six machine
motors across the two AZD3A controllers. It never publishes directly to
raw ros2_control topics.

## Safety boundary

- The on-screen Motor 2 stop is a commanded, ramped stop. It is not an
  emergency stop; keep the physical power cutoff accessible.
- Motor 1 is limited to 0.0996..200 mm above its provisional lower-end zero
  (half of the 400 mm actuator stroke). Its required velocity is 15 mm/s with
  the retained symmetric 5 mm/s^2 software acceleration/deceleration ramp.
- Motor 2 is limited to its validated 250 rpm machine requirement.
- Motors 3 and 4 are limited to 0..15 mm in 0.001 mm increments. Both use the
  required 8 mm/s velocity and symmetric 50 mm/s^2 software ramps. Motor 3 uses
  the custom `motor3_position` interface on slave 0 Axis 3.
- Motor 6 is limited to +/-50 output rpm with a 2 rpm/s ramp and a 0.5-second
  watchdog. Positive RPM is the physically verified forward conveyor
  direction. The GUI accepts the operator command in belt mm/s: 50 rpm on the
  verified 30 mm pulley equals approximately 78.54 mm/s.
- Motor 5 is limited to +/-90 degrees from an operator-defined runtime origin,
  at 5 rpm with a symmetric 5 rpm/s ramp. Its CW/CCW motion buttons remain
  locked until **Set current position as zero** is pressed after each launch.
- The combined launch owns each AZD3A slave once and operates Motors 1, 2, and
  3 through separate guarded interfaces on slave 0. Slave 1 similarly combines
  Motor 4 Axis 1 position and Motor 6 Axis 3 velocity interfaces.
  Commands may be issued step by step or concurrently when the machine sequence
  eventually requires it.

## Build and run

```bash
cd ~/kyutech/azd3a_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select motor_controller
source install/setup.bash
```

Start the combined backend and GUI:

```bash
ros2 launch motor_controller azd3a_motor1_motor2_gui.launch.py
```

The filename predates Motor 4 and Motor 6 integration. It is retained for
compatibility but now starts all six motors.

Select **Show motion details** below the motor panels to view each configured
range, velocity, acceleration, and deceleration. These values are read-only in
the GUI and mirror the guarded backend settings. A motor's buttons stay
disabled until its guarded backend subscriber is present.

Motor 1 entries are in millimetres. The GUI aligns them to the 0.0012 mm
encoder count before publishing. Motor 2 entries are in rpm; while rotation is
active, the GUI refreshes the command so the existing 0.5-second watchdog does
not stop it. Motor 3 and Motor 4 entries are absolute millimetres from their ABZO
coordinate. Motor 6 entries are belt speed in mm/s; the GUI converts them to
gearbox-output rpm internally and displays both values in feedback. Closing the GUI publishes repeated Motor 2 and
Motor 6 zero commands; position axes remain holding their last guarded targets
until the backend shuts down.

## Motor 4 troubleshooting record

The combined two-slave launch has been physically validated for step-by-step
Motor 1, Motor 2, and Motor 4 control. Expected EtherCAT state is both slaves
OP with a complete domain working counter (observed 6/6).

If the master repeatedly alternates `0x08` and `0x0C`, inspect individual
slaves and AL register `0x0134`. Code `0x001B` is a SyncManager watchdog and
means a slave requested into OP is not receiving its cyclic output PDO. Do not
raise motor speed until the domain working counter is complete.

If the Motor 4 guard accepts a target and the raw topic changes, but `0x607A`
stays equal to `0x6064`, check for a finite CSP `position_startup_tolerance`
on the Motor 4 module. The combined configuration intentionally delegates
startup synchronization to `azd3a_motor4_position_guard.py`; reintroducing the
plugin latch can permanently override valid guarded commands during concurrent
controller startup.

Full engineering record: `MOTOR4_COMMISSIONING_POSTMORTEM.md`.

## Validation status (2026-09-22)

- Motors 1, 2, and 4 have been physically controlled from the combined GUI.
- Motor 6 standalone commissioning passed at +1 output rpm: smooth forward
  motion, watchdog stop, zero final velocity, and no drive alarm.
- Motor 6 is now software-integrated into the combined GUI with a 50 rpm
  (78.54 mm/s) guard, 2 rpm/s ramp, live RPM/belt-speed display, and global
  rotary stop.
- The combined Motor 6 GUI path and expanded speed range have passed software
  tests and build/launch loading, but still require staged physical validation.
  Begin at the GUI default 10 mm/s, then increase in deliberate steps before
  trying 78.54 mm/s.
- Repository test result at closeout: 21 tests passed; combined xacro expansion,
  ROS package build, and launch-description loading passed.
- Motor 3 standalone commissioning passed its 2 mm round trip at 8 mm/s and
  50 mm/s^2. The driver now has a third CiA-402 state machine, the combined
  backend maps slave 0 Axis 3 through `motor3_position`, and the GUI is unlocked
  behind subscriber/feedback guards. Software validation passed 23 repository
  tests plus 51 driver test results (zero failures). Combined physical testing
  must begin with feedback inspection and a 0.1 mm move.
- The first combined Motor 3 GUI test passed: 0.2732 mm start to a 0.373 mm
  target settled at 0.3727 mm with no alarm, then Return to 0 mm settled at
  exactly 0 counts with no alarm. The operator confirmed smooth physical motion
  in the required forward direction; combined Motor 3 commissioning is complete.
