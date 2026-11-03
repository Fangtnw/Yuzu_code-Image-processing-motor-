> Current integrated physical port wiring: [AXIS_WIRING.md](AXIS_WIRING.md). Motor numbers in this document identify machine roles.

# Yuzu Peeler Operator GUI

The operator panel controls guarded public interfaces for all six machine
motors across the two AZD3A controllers. It never publishes directly to
raw ros2_control topics. It shows per-axis ROS feedback freshness and guard
topic connections and drive statuswords. Motion readiness requires fresh
feedback and all six drives in Operation Enabled. This software indication
is not an independent safety interlock or a substitute for a hardware stop.
The external GUI topic names, message types, units, and conveyor request
protocol are specified in `MOTOR_ROS2_INTERFACE.md`.

## Safety boundary

- The on-screen rotary stop sends controlled zero-speed commands to Motors 2
  and 6. It is not an emergency stop; keep the physical power cutoff
  accessible.
- Motor 1's backend and GUI range is 0..400 mm. Its 600 mm/s speed cap matches
  the actuator's published maximum; its symmetric 450 mm/s²
  acceleration/deceleration ramps reach or stop from that cap in about 1.33 s on a long
  enough move. The catalog page does not give a maximum acceleration rating.
  The operator has validated 100 mm/s physically, so the cap and new ramp need
  staged validation on the loaded assembly. A rest-to-rest move at the full
  600 mm/s cap needs about 800 mm to accelerate and brake, so it cannot reach
  that cap and stop within the guarded 400 mm stroke.
- Motor 1 now uses a provisional top home: 0 mm is raw encoder count 2,001,
  and positive targets move downward. This direction and the assembled travel
  need a small supervised physical validation before full-range operation.
- Motor 2 is capped at 416 rpm, its published output maximum. The editable
  speed field defaults to 374.4 rpm (90%); enter 416 rpm for 100%.
  The default acceleration/deceleration is 832 rpm/s, reaching the cap in
  0.5 s. Acc/dec entries remain adjustable, but guards reject values too low
  for the requested speed or above the configured ceiling; these are software
  ramps, not manufacturer ratings.
- Motors 3 and 4 are guarded to 0..20 mm (the DR28 catalog stroke is 30 mm) in
  0.0001 mm hardware-count steps. Both use the
  40 mm/s velocity and symmetric 80 mm/s^2 ramps, reaching the cap in 0.5 s.
  The acceleration is below the DR28's published 200 mm/s² maximum. Motor 3 uses
  the custom `motor3_position` interface on slave 1 local Axis 2
  (physical Axis 5).
- Motor 6 indexes 140 mm per step and is limited to 60 output rpm, about
  133.83 mm/s at the calibrated conversion. The GUI requests 100.0 mm/s by
  default with a 120 rpm/s software
  ramp; the configured speed is reached within 0.5 s;
  acc/dec entries are constrained to satisfy that time.
  The 0.5-second watchdog remains, refreshed by index heartbeats during a
  step or speed commands during manual running. Backend indexing requires
  position, velocity and enabled status together, no older than 0.1 s. Positive RPM
  is the physically verified forward conveyor direction.
- Motor 5 uses a project limit of +/-180 degrees from an operator-defined runtime
  origin at 150 rpm, its published output maximum. Its 300 rpm/s ramp is a
  software profile, reaching cap in 0.5 s when the move is long
  enough. The encoder/drive has no intrinsic angular travel stop, so
  the mechanical fixture must define any wider safe range. Before zero capture, the `<`/`>`
  step buttons provide guarded manual jogging; stop the mechanism, then press
  **Set current position as zero** before using typed angle or return commands.
  After zero capture, `<` and `>` step the signed target through zero across
  the full -180°..+180° range. The CW/CCW selection follows the target sign;
  the angle entry remains a nonnegative magnitude. The buttons step from the
  displayed target, so monitor live feedback before issuing another move.
- The combined launch owns each AZD3A slave once. Slave 0 contains physical
  Axes 1/2/3 (Motors 1/2/6); slave 1 contains physical Axes 4/5/6
  (Motors 4/3/5).
Commands may be issued step by step or concurrently when the machine sequence
eventually requires it.

The 0.5 s acceleration/deceleration time is the ramp to/from a requested speed,
not a guarantee that every positioning move reaches that speed. Short moves
must decelerate early. Motor 6's 140 mm step has sufficient distance to
reach the current speed cap and brake with its 120 rpm/s ramp. Motors 2–5 are intended to
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
are in rpm (416 rpm cap); its acceleration and deceleration fields
default to 832 rpm/s and remain operator-adjustable within the guard. These
are software profiles, not manufacturer acceleration ratings. While rotation is active, the GUI
refreshes the command so the existing 0.5-second watchdog does
not stop it. Motor 3 and Motor 4 entries are absolute millimetres from their ABZO
coordinate. Motor 6 entries and feedback are belt speed in mm/s, with a
read-only 140 mm index step. The first step captures the stopped position
as the grid reference; subsequent targets use the index number and that same
reference, without accumulating previous stop errors. The backend generates
an absolute CSP position trajectory internally. Continuous Motor 6 manual
speed is disabled; use the **CSP jog (mm)** buttons for realignment, then
reset the reference at the visible fixture mark. **Resume interrupted step** returns to the retained target
after a stop; it does not add another step. It uses the current Manual control
Motor 6 speed and Acc/Dec fields, allowing a slower retry of an interrupted
index without changing its absolute fixture target. **Reset step reference here**
deliberately replaces the grid at a stopped position. A backend
restart also requires a new grid. The prominent rotary stop sends repeated controlled
zero-speed commands to Motors 2 and 6. Closing the GUI publishes repeated Motor 2 and
Motor 6 zero commands; position axes remain holding their last guarded targets
until the backend shuts down.

The Operation sequence tab has six actions: Motor 6 index, camera/fixed-data
snapshot, Motor 1 pierce, Motor 1 lift, coordinated Motors 3/4/5 contour peel,
and Motor 1 return home. **Run full sequence** starts at the conveyor index
and automatically runs the remaining actions one by one. It waits for the
conveyor's stopped target before capturing new camera messages, then uses
that one snapshot for both Motor 1 moves and the contour peel. If camera
messages do not arrive within 10 seconds, or any stage fails, the runner
stops without sending the next target. **Execute next step** advances only after the
feedback-checked Motor 1 or peel command succeeds; conveyor indexing still
requires stopped completion before advancing. **Execute selected step** runs
any chosen action independently, then sets **Execute next step** to the
following action after successful completion. A failed action leaves the
sequence pointer where it was. The
separate Motor 2 rotation controls remain manual and are not part of Step 3.

Choose **Camera snapshot** and press **Capture snapshot** while the fruit is
visible for individual testing, or use the full runner to capture automatically
after the conveyor stops. The GUI
requires fresh `/yuzu/y_offset_mm` and `/yuzu/contour_point_offsets` samples
close in receipt time, validates the 72-ray contour and all slide targets,
then saves both values in the GUI for this cycle. The same Motor 1 offset,
frame reference, home position, lift distance, and allowance are used for
both pierce and lift. Step 3 passes the saved contour to the coordinator via
a temporary JSON file, so loss of camera view after lifting does not change
the feed targets. Resetting the sequence clears the snapshot; restarting the
GUI also clears it. The camera messages lack a fruit ID or capture timestamp,
so the operator must ensure both samples belong to the visible fruit.

For a demo without the camera PC, choose **Fixed demo values**, set the signed
Y offset and 5° inset contour radius, and capture a synthetic circular
snapshot. The default offset is −30 mm and default inset radius is 17 mm;
the Step 3 calculation restores the detector's 3 mm inset and targets the
outer surface. The other editable fields include the Motor 1 frame encoder
(default 204 mm), lift (default 150 mm), blade center distances (default
35 mm each), and Motor 5 sweep speed (default 10°/s). Motor 6's index distance
remains read-only from its backend. The Operation Sequence tab has its own
Motor 6 speed (default 100.0 mm/s) and Acc/Dec fields (120 rpm/s each), so
the full run does not inherit values left in Manual control. A manually selected lift still requires
Motor 1 to be at the calculated pierce target; selected pierce requires Motor 1
at its configured home position within 0.1 mm. The GUI reports the measured
start position if that check fails. A selected peel still requires
Motors 3/4 at zero and Motor 5 at its runtime zero. The GUI reports command
success only after the child command verifies feedback.
Each operation step unlocks when its required motors have fresh feedback and
CiA-402 Operation Enabled status (`(statusword & 0x006F) == 0x0027`). This lets
an operator test Motor 1 or Motors 3/4/5 without requiring the other drives.
The banner still shows readiness for all six mapped motors.

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

The requested 140 mm is a nominal value for
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
