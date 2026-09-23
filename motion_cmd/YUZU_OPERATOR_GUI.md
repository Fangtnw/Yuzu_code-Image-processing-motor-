# Yuzu Peeler Operator GUI

The operator panel controls guarded public interfaces for all six machine
motors across the two AZD3A controllers. It never publishes directly to
raw ros2_control topics. It shows per-axis ROS feedback freshness and guard
topic connections; it does not receive EtherCAT state, drive alarms, or an
independent drive-ready indication.

## Safety boundary

- The on-screen rotary stop sends controlled zero-speed commands to Motors 2
  and 6. It is not an emergency stop; keep the physical power cutoff
  accessible.
- Motor 1's backend and GUI range is now 0..400 mm across the full actuator
  stroke. Return-to-0 commands are allowed; its required velocity is 15 mm/s with a
  symmetric 15 mm/s^2 software acceleration/deceleration ramp.
- Motor 2 is limited to its validated 250 rpm machine requirement.
- Motors 3 and 4 are guarded to 0..20 mm (the DR28 catalog stroke is 30 mm) in
  0.0001 mm hardware-count steps. Both use the
  required 8 mm/s velocity and symmetric 50 mm/s^2 software ramps. Motor 3 uses
  the custom `motor3_position` interface on slave 0 Axis 3.
- Motor 6 is limited to 90 mm/s in the GUI (about 57.3 rpm); the drive maximum
  is 60 rpm, approximately 94.25 mm/s on the 30 mm pulley. It uses a 250 rpm/s ramp and a 0.5-second
  watchdog. Positive RPM is the physically verified forward conveyor
  direction. The GUI accepts the operator command in belt mm/s: 50 rpm on the
  verified 30 mm pulley equals approximately 78.54 mm/s.
- Motor 5 uses a project limit of +/-180 degrees from an operator-defined runtime
  origin at 20 rpm. The encoder/drive has no intrinsic angular travel stop, so
  the mechanical fixture must define any wider safe range. Before zero capture, the `<`/`>`
  step buttons provide guarded manual jogging; stop the mechanism, then press
  **Set current position as zero** before using typed angle or return commands.
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

The machine banner summarizes guard topic connections and fresh axis feedback.
Each motor panel shows its own feedback state. Motion buttons remain disabled
until both the guarded backend subscriber and that motor's fresh feedback are
present. Fresh feedback does not prove the drive is alarm-free or enabled.
If Motor 2 or Motor 6 feedback becomes stale during rotation, the GUI stops
refreshing that speed command and sends repeated zero-speed commands. The
controlled stop buttons remain available while their command backend is
connected. Linear depth panels include 0..15 mm position bars; Motor 5 includes
a runtime-zero-relative ±180° bar. The GUI reports command transmission and
live feedback, but the current ROS status topics do not report target-reached
or motion-complete events.
Select **Show motion details** below the motor panels to view each configured
range, velocity, acceleration, and deceleration. These values are read-only in
the GUI and mirror the guarded backend settings.

Motor 1 entries are in millimetres. The GUI aligns them to the 0.0012 mm
encoder count before publishing and uses a 1 mm default step. Motor 2 entries are in rpm
(200 rpm by default); its acceleration and deceleration fields default to
500 rpm/s and may be reduced for gentler commissioning. This 500 rpm/s value
is an empirical software ceiling for speed matching, not a manufacturer rating. While rotation is active, the GUI
refreshes the command so the existing 0.5-second watchdog does
not stop it. Motor 3 and Motor 4 entries are absolute millimetres from their ABZO
coordinate. Motor 6 entries and feedback are belt speed in mm/s (30 mm default
distance step); the GUI also
supports relative distance steps using encoder position feedback, converts
commands to gearbox-output rpm internally, and keeps rpm out of the
operator display. The prominent rotary stop sends repeated controlled
zero-speed commands to Motors 2 and 6. Closing the GUI publishes repeated Motor 2 and
Motor 6 zero commands; position axes remain holding their last guarded targets
until the backend shuts down.

The Operation sequence tab provides guarded, operator-advanced stages matching
the peeling sequence: Motor 6 positioning, Motor 1 approach, Motor 5 peeler
positioning, rotation/feed, Motor 2 stop after step 4.3, gripping, Motor 1 home,
release, and cycle wait.
Motor 1/3/4 operation distances and Motor 2 speed are editable before running.
The default Motor 1 operation approach/home positions are 300 mm and 400 mm;
the complete step list remains visible with only the active step highlighted.
The GUI unlocks motion only after all six mapped CiA-402 status words report
Operation Enabled (`0x0567` pattern); otherwise the banner identifies the
motors still starting or faulted.

Motor 5 zero capture requires fresh feedback and an operator confirmation that
the mechanism is stationary and at the intended reference pose. Since the
available interface does not expose an independent drive-stopped signal, the
operator remains responsible for confirming that condition.

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
  (78.54 mm/s) guard, 25 rpm/s ramp, live belt-speed display, and global
  rotary stop.
- The combined Motor 6 GUI path and expanded speed range have passed software
  tests and build/launch loading, but still require staged physical validation.
  Begin at the GUI default 10 mm/s, then increase in deliberate steps before
  trying 78.54 mm/s. The GUI defaults are Motor 3/4 step 1 mm, Motor 5 angle 0°
  with a 90° step, and Motor 6 belt speed 50 mm/s.
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
