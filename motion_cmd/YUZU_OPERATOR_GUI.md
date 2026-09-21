# Yuzu Peeler Operator GUI

The operator panel controls the guarded public interfaces for machine Motors
1, 2, 4, and 6 across the two AZD3A controllers. It never publishes directly to
raw ros2_control topics.

## Safety boundary

- The on-screen Motor 2 stop is a commanded, ramped stop. It is not an
  emergency stop; keep the physical power cutoff accessible.
- Motor 1 is limited to 0.0996..5 mm above its provisional lower-end zero.
- Motor 2 is limited to its validated 250 rpm machine requirement.
- Motor 4 is limited to 0..15 mm in 0.001 mm increments. Its guard chooses a
  conservative profile from the commanded distance: 2 mm/s through 1 mm,
  5 mm/s through 5 mm, and 10 mm/s above 5 mm.
- Motor 6 is limited to +/-5 output rpm with a 2 rpm/s ramp and a 0.5-second
  watchdog. Positive RPM is the physically verified forward conveyor
  direction. At 5 rpm, the 30 mm pulley gives approximately 7.85 mm/s belt
  speed.
- The combined launch owns the AZD3A slave once and operates Motor 1 and Motor
  2 through separate guarded interfaces. Its second slave similarly combines
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

Motor 1 entries are in millimetres. The GUI aligns them to the 0.0012 mm
encoder count before publishing. Motor 2 entries are in rpm; while rotation is
active, the GUI refreshes the command so the existing 0.5-second watchdog does
not stop it. Motor 4 entries are absolute millimetres from the actuator's ABZO
coordinate. Motor 6 entries are gearbox-output rpm, and the GUI shows both rpm
and calculated belt speed. Closing the GUI publishes repeated Motor 2 and
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
