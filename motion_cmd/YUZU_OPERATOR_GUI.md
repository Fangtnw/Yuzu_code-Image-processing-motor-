# Yuzu Peeler Operator GUI

The operator panel controls guarded public interfaces for all six machine
motors across the two AZD3A controllers. It never publishes directly to
raw ros2_control topics. It shows per-axis ROS feedback freshness and guard
topic connections and drive statuswords. Motion readiness requires fresh
feedback and all six drives in Operation Enabled. This software indication
is not an independent safety interlock or a substitute for a hardware stop.

## Safety boundary

- The on-screen rotary stop sends controlled zero-speed commands to Motors 2
  and 6. It is not an emergency stop; keep the physical power cutoff
  accessible.
- Motor 1's backend and GUI range is 0..400 mm. Its 480 mm/s speed cap is 80%
  of the actuator's 600 mm/s published maximum; its symmetric 480 mm/s²
  acceleration/deceleration ramps reach or stop from that cap in 1 s on a long
  enough move. The catalog page does not give a maximum acceleration rating.
  The operator has validated 100 mm/s physically, so the cap and new ramp need
  staged validation on the loaded assembly. A rest-to-rest move at the full
  480 mm/s cap needs about 480 mm to accelerate and brake, so it cannot reach
  that cap and stop within the guarded 400 mm stroke.
- Motor 2 is capped at 332.8 rpm, 80% of its 416 rpm published output maximum.
  The default acceleration/deceleration is 665.6 rpm/s, reaching the cap in
  0.5 s. Acc/dec entries remain adjustable, but guards reject values too low
  for the requested speed or above the configured ceiling; these are software
  ramps, not manufacturer ratings.
- Motors 3 and 4 are guarded to 0..20 mm (the DR28 catalog stroke is 30 mm) in
  0.0001 mm hardware-count steps. Both use the
  32 mm/s velocity and symmetric 64 mm/s^2 ramps, reaching the cap in 0.5 s.
  The acceleration is below the DR28's published 200 mm/s² maximum. Motor 3 uses
  the custom `motor3_position` interface on slave 0 Axis 3.
- Motor 6 indexes 140.825 mm per step and is limited to 48 rpm (75.4 mm/s belt speed), 80% of its 60 rpm
  output maximum. The GUI defaults to that speed and a 96 rpm/s software ramp,
  reaching cap in 0.5 s; acc/dec entries are constrained to satisfy that time.
  The 0.5-second watchdog remains, refreshed by index heartbeats during a
  step or speed commands during manual running. Backend indexing requires
  position, velocity and enabled status together, no older than 0.1 s. Positive RPM
  is the physically verified forward conveyor direction.
- Motor 5 uses a project limit of +/-180 degrees from an operator-defined runtime
  origin at 120 rpm, 80% of its 150 rpm output maximum. Its 240 rpm/s ramp is a
  software profile, reaching cap in 0.5 s when the move is long
  enough. The encoder/drive has no intrinsic angular travel stop, so
  the mechanical fixture must define any wider safe range. Before zero capture, the `<`/`>`
  step buttons provide guarded manual jogging; stop the mechanism, then press
  **Set current position as zero** before using typed angle or return commands.
- The combined launch owns each AZD3A slave once and operates Motors 1, 2, and
  3 through separate guarded interfaces on slave 0. Slave 1 similarly combines
  Motor 4 Axis 1 position and Motor 6 Axis 3 cyclic synchronous position interfaces.
Commands may be issued step by step or concurrently when the machine sequence
eventually requires it.

The 0.5 s acceleration/deceleration time is the ramp to/from a requested speed,
not a guarantee that every positioning move reaches that speed. Short moves
must decelerate early. Motor 6's 140.825 mm step has sufficient distance to
reach the 48 rpm cap and brake with its 96 rpm/s ramp. Motors 2–5 are intended to
run simultaneously in part of the sequence; staged combined-load validation
is required before using all their caps together. Exact official catalog
sources and calculations are in `vendor/oriental_motor/MOTOR_SPEED_SOURCES.md`.

## Build and run

```bash
cd ~/yuzu_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select motor_controller
source install/setup.bash
```

Start the combined backend and GUI:

```bash
ros2 launch motor_controller yuzu_peeler.launch.py
```

This launch starts the integrated backend and GUI for all six motors.

The machine banner summarizes guard topic connections and fresh axis feedback.
Each motor panel shows its own feedback state. Motion buttons remain disabled
until both the guarded backend subscriber and that motor's fresh feedback are
present. Fresh feedback does not prove the drive is alarm-free or enabled.
If Motor 2 or Motor 6 feedback becomes stale during rotation, the GUI stops
refreshing that speed command and sends repeated zero-speed commands. The
controlled stop buttons remain available while their command backend is
  connected. Linear depth panels include 0..20 mm position bars; Motor 5 includes
a runtime-zero-relative ±180° bar. The GUI reports command transmission and
live feedback. Motor 6 also reports backend indexing, interrupted, and
settled-completion states; the other motors do not report motion-complete events.
Select **Show hardware & motion details** below the motor panels to view each configured
range, velocity, acceleration, and deceleration. These values are read-only in
the GUI and mirror the guarded backend settings.

Motor 1 entries are in millimetres. The GUI aligns them to the 0.0012 mm
encoder count before publishing and uses a 1 mm default step. Motor 2 entries
are in rpm (332.8 rpm default cap); its acceleration and deceleration fields
default to 665.6 rpm/s and remain operator-adjustable within the guard. These
are software profiles, not manufacturer acceleration ratings. While rotation is active, the GUI
refreshes the command so the existing 0.5-second watchdog does
not stop it. Motor 3 and Motor 4 entries are absolute millimetres from their ABZO
coordinate. Motor 6 entries and feedback are belt speed in mm/s, with a
read-only 140.825 mm index step. The first step captures the stopped position
as the grid reference; subsequent targets use the index number and that same
reference, without accumulating previous stop errors. The backend generates
an absolute CSP position trajectory internally. Continuous Motor 6 manual
speed is disabled; use the **CSP jog (mm)** buttons for realignment, then
reset the reference at the visible fixture mark. **Resume interrupted step** returns to the retained target
after a stop; it does not add another step. **Reset step reference here**
deliberately replaces the grid at a stopped position. A backend
restart also requires a new grid. The prominent rotary stop sends repeated controlled
zero-speed commands to Motors 2 and 6. Closing the GUI publishes repeated Motor 2 and
Motor 6 zero commands; position axes remain holding their last guarded targets
until the backend shuts down.

The Operation sequence tab provides guarded, operator-advanced stages matching
the peeling sequence: Motor 6 positioning, Motor 1 approach, Motor 5 peeler
positioning, rotation/feed, Motor 2 stop after step 4.3, gripping, Motor 1 home,
release, and cycle wait.
Motor 1/3/4 operation distances and Motor 2 speed are editable before running.
Motor 6's distance is read-only and comes from the backend. Placement cannot
advance until the requested conveyor index reports stopped completion. No
fixture-mark confirmation or sensor workflow is included in this version.
The default Motor 1 operation approach/home positions are 300 mm and 400 mm;
the complete step list remains visible with only the active step highlighted.
The GUI unlocks motion only after all six mapped CiA-402 status words report
Operation Enabled (`(statusword & 0x006F) == 0x0027`); otherwise the banner identifies the
motors still starting or faulted.

Motor 5 zero capture requires fresh feedback and an operator confirmation that
the mechanism is stationary and at the intended reference pose. Since the
available interface does not expose an independent drive-stopped signal, the
operator remains responsible for confirming that condition.

### Motor 6 accuracy and commissioning

The distance loop runs in the guard at 200 Hz, independently of the GUI's
100 ms display refresh. It retains the absolute target while braking and
approaching, and declares completion only after the error is within 0.02 mm
of encoder-derived travel and measured motion has settled. Status shows the
remaining signed encoder error. Heartbeat/feedback loss, a missed control
deadline or a movement timeout interrupts the step instead of reporting success.

The requested 140.825 mm is a rounded, drawing-derived nominal value for
12 fixtures. It has not been physically calibrated. Count scaling, belt slip,
stretch, gearbox backlash and fixture placement can affect actual belt travel;
the software settling threshold is not a conveyor accuracy guarantee. Check
marked-belt travel and repeated full circuits on the real machine before using
the fixture load. See `MOTOR6_INDEXING.md` for validation and calibration notes.

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

## Historical commissioning record (through 2026-09-22)

The values below record the system state at that date; current limits and
profiles are listed in the Safety boundary above and in the source register.

- Motors 1, 2, and 4 have been physically controlled from the combined GUI.
- Motor 6 standalone commissioning passed at +1 output rpm: smooth forward
  motion, watchdog stop, zero final velocity, and no drive alarm.
- Motor 6 was initially software-integrated into the combined GUI with a 50 rpm
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
  50 mm/s^2 before the supervisor-requested 80% speed update. The driver now has a third CiA-402 state machine, the combined
  backend maps slave 0 Axis 3 through `motor3_position`, and the GUI is unlocked
  behind subscriber/feedback guards. Software validation passed 23 repository
  tests plus 51 driver test results (zero failures). Combined physical testing
  must begin with feedback inspection and a 0.1 mm move.
- The first combined Motor 3 GUI test passed: 0.2732 mm start to a 0.373 mm
  target settled at 0.3727 mm with no alarm, then Return to 0 mm settled at
  exactly 0 counts with no alarm. The operator confirmed smooth physical motion
  in the required forward direction; combined Motor 3 commissioning is complete.
