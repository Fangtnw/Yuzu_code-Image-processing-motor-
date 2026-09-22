# AZD3A-KED Connected Hardware

This records the physical devices connected during the one-drive bring-up.
Model strings should be checked against the physical labels before scaling or
motion limits are treated as final.

Axis numbering here follows the current AZD3A wiring. As of 2026-09-21, the
drive has been rewired to follow the requirement sequence: logical Motor 1 is
on Axis 1 and logical Motor 2 is on Axis 2. Axis 3 is disconnected. The old
Axis 1 `DR28T1A03-AZAKR` commissioning setup is no longer connected.

The final machine has six logical motors and is expected to use two
three-axis AZD3A-KED controllers. Software must therefore identify a joint by
its machine role first and resolve the EtherCAT slave and local drive axis from
configuration. Do not encode assumptions such as `axis3 == Motor 5` into the
final sequence controller.

## EtherCAT drive

| Item | Value |
| --- | --- |
| Model | AZD3A-KED |
| Axes | 3 |
| Vendor ID | `0x000002BE` |
| Product ID | `0x000013AF` |
| Revision | `0x01110301` |
| ESI | `vendor/oriental_motor/ORIENTALMOTOR_AZDxA-KED_rev0301.xml` |

## Axis assignments

| AZD3A axis | Connected device | Motion type | Verification |
| --- | --- | --- | --- |
| Axis 1 | `AZM46AK`, parameterized for `EZSM3LD040AZAK` | Linear slide, machine Motor 1 | Label photo and live A=1/B=1 values confirmed |
| Axis 2 | `AZM46AK-FC7.2UA` | Rotary speed / spin-stop, machine Motor 2 | User-corrected physical model; verify scaling before motion |
| Axis 3 | Disconnected | No current motor | Previous Motor 5 commissioning wiring removed |

These are temporary bring-up ports on the currently available AZD3A-KED:

| Current local port | Temporary connected machine role |
| --- | --- |
| AZD3A slave 0, Axis 1 | Motor 1 (`AZM46AK` / `EZSM3LD040AZAK`) |
| AZD3A slave 0, Axis 2 | Motor 2 (`AZM46AK-FC7.2UA`) |
| AZD3A slave 0, Axis 3 | Motor 3, DR28T1A03-AZAKR, 10,000 counts/mm; standalone commissioning passed |

This table is commissioning evidence, not the final harness assignment. When
the remaining motors and second AZD3A-KED arrive, record an explicit mapping
for each logical `motor_1` through `motor_6` to `{slave alias, local axis}`.
Prefer persistent EtherCAT aliases over chain position if the installation
allows it, because physical slave position can change when cabling changes.

Motor 4 commissioning now uses AZD3A slave 1, local Axis 1 (CN7). Its confirmed
`DR28T1A03-AZAKR` scaling is 10,000 counts/mm with A=1/B=1. A +0.100 mm
startup-relative round trip completed without alarm: raw -37 to approximately
960 and back to -34. The guarded follow-up envelope is +/-0.500 mm from the
startup feedback position at 0.5 mm/s.

The subsequent +0.500 mm command reached raw 4,966 exactly and remained at
alarm `0x0000`. A guarded -0.500 mm recovery after restart returned exactly to
raw -34 with no alarm. Both EtherCAT slaves were OP and the configured domain
working counter was 3/3 during the diagnostic check. This verifies Motor 4's
address, scaling, bidirectional motion, and repeatability over 0.500 mm.

The first slave-1-only launch caused upstream slave 0 to alternate EtherCAT AL
states with code `0x001B` (SyncManager watchdog), although slave 1 stayed OP at
WorkingCounter 3/3. The revised launch cyclically maps slave 0 through a passive
`GenericEcSlave` with controlword/mode held at zero; it must not enable Motors
1/2. Motor 4's guarded test speed is now 2.0 mm/s. Motor 3 speed remains pending
because slave 0 Axis 3 is still recorded as disconnected.

The combined GUI/backend now owns both physical AZD3A slaves: Motors 1/2 use
slave 0 and Motor 4 uses slave 1 Axis 1. At that commissioning stage, Motor 4
exposed guarded absolute targets from 0..15 mm with distance-adaptive 2/5/10
mm/s profiles. This removes
the passive-keepalive case from combined operation because both slaves have
real cyclic PDO mappings. Runtime validation was pending at that stage.

Runtime validation is now complete for step-by-step combined control: the
operator confirmed Motors 1 and 2 remained controllable and Motor 4 physically
moved after its finite CSP plugin startup latch was removed. Motor 4 startup
safety is enforced by the feedback-aware application guard, which rejects
commands until valid feedback exists. See
`MOTOR4_COMMISSIONING_POSTMORTEM.md` for the root-cause and validation record.

Motor 6 is connected to AZD3A slave 1, local Axis 3. Its motor is
`AZM46AK-PS50` (50:1 planetary gear) and its conveyor is a MISUMI
`SVKA-150-795-25-...` with a 30 mm drive pulley. Live A=1/B=1 values give
500,000 counts per output revolution. After connection and power cycle, Axis 3
reported alarm/error `0x0000`, status `0x0270`, and raw position 2,280,574.
The initial guarded test is capped at 1 output rpm (about 1.5708 mm/s belt
speed) with a 1 rpm/s ramp and 0.5-second command watchdog.

The first physical 1 rpm commissioning run completed successfully. Positive
RPM moved the conveyor smoothly in the operator-confirmed **forward**
direction. Raw position increased from 2,280,574 to 2,358,024 counts, a change
of 77,450 counts (0.1549 output revolution, approximately 14.6 mm of belt
travel with the 30 mm pulley). After the timed command ended, feedback showed
zero velocity and Axis 3 alarm `0x4040:03=0x0000`. The initial direction,
scaling, watchdog stop, and 1 rpm commissioning profile are therefore
physically validated.

### Motor 6 and conveyor specification record

This table preserves the hardware data collected during commissioning. Source
types are explicit so that a catalog value is not confused with a live drive
setting or a software safety limit.

| Item | Value | Evidence |
| --- | --- | --- |
| Motor model | `AZM46AK-PS50` | Physical nameplate photo |
| Manufacturer / family | Oriental Motor, AZ Series closed-loop stepper with mechanical absolute encoder | Official product page |
| Frame size | 42 mm (1.65 in) | Official product page |
| Gear type and ratio | Planetary, 50:1 | Model suffix and official product page |
| Nameplate winding rating | 2.04 VDC, 1.48 A | Physical nameplate photo |
| Nameplate step angle | 0.0072 degrees/step | Physical nameplate photo |
| Motor length | 121.5 mm (4.78 in) | Official product page |
| Holding torque | 3 N m (420 oz-in) | Official product page |
| Holding torque at standstill | 3 N m (425 oz-in) | Official product page |
| Permissible output speed | 0..60 rpm | Official product page |
| Backlash | 15 arc min (0.25 degrees) | Official product page |
| Stop-position accuracy | +/-4 arc min (+/-0.067 degrees) | Official product page |
| Electromagnetic brake | Not equipped | Official product page |
| Rotor inertia | `55e-7 kg m^2` | Official product page |
| Conveyor label | `SVKA-150-795-25-NV-NM-NH-W-R-...` | Physical conveyor label photo; trailing option text was outside/unclear in the photo |
| Conveyor family | MISUMI SV Series, SVKA end-drive flat-belt conveyor, two-groove frame | Official MISUMI catalog |
| Belt label | `HBLTWH150-1.68` | Physical conveyor label photo |
| Belt width | 150 mm | Conveyor and belt label model strings |
| Pulley-center distance | 795 mm | Conveyor label model string |
| Drive pulley diameter | 30 mm | Official MISUMI SVKA catalog and calculation table |
| Live electronic gearing | A=1, B=1 | SDO `0x7091:01/02` read from slave 1 Axis 3 |
| Live software conversion | 500,000 drive counts/output revolution | Verified commissioning configuration; not the catalog's selectable pulse-resolution figure |
| Current GUI speed cap | 50 output rpm / 78.54 mm/s belt speed | Project software safety limit below the 60 rpm hardware maximum |

The conversions used by the Motor 6 ROS configuration are:

```text
500,000 counts/output revolution
79,577.47154594767 counts/output radian
0.000012566370614359173 output radian/count

belt travel/output revolution = pi * 30 mm = 94.2477796077 mm
belt speed at 1 output rpm = pi * 30 / 60 = 1.57079632679 mm/s
belt distance/count = pi * 30 / 500,000 = 0.000188495559215 mm/count
```

The Oriental Motor product page also lists a selectable-resolution example of
0.0072 degrees/pulse at 1,000 P/R. That catalog pulse setting must not replace
the live 500,000-count/output-revolution ROS conversion without a new drive
configuration and a measured scaling test.

As of 2026-09-22, Motor 6 is integrated with Motor 4 through one composite
slave-1 configuration. Local Axis 1 remains Motor 4 CSP position and local
Axis 3 is Motor 6 CSV velocity; this avoids two ROS hardware instances
competing for AZD3A #2. The operator guard is now capped at 50 output rpm with
a 2 rpm/s ramp and 0.5-second watchdog. This corresponds to approximately
78.54 mm/s belt speed. The standalone +1 rpm physical test is validated; the
expanded combined-GUI speed range requires staged physical validation.

### Updated linear-axis motion requirements (2026-09-22)

- Motor 1 required velocity is 15 mm/s. Its guarded upper position is now
  200 mm, half of the `EZSM3LD040AZAK` 400 mm catalog stroke; the lower bound
  remains 0.0996 mm above the provisional lower-end zero. The retained
acceleration/deceleration ramp is symmetric at 15 mm/s^2 for the 15 mm/s
operating speed.
- Motors 3 and 4 use 8 mm/s motion with symmetric 50 mm/s^2
  acceleration/deceleration over guarded 0..15 mm ranges. Motor 3 is slave 0
  Axis 3 and Motor 4 is slave 1 Axis 1. Motor 3's scaling, direction, alarm-free
  stopping, and 2 mm round trip have been physically commissioned. The plugin
  now supports a tertiary CiA-402 state machine, and Motor 3 is exposed through
  the combined GUI's guarded custom position interface.

Motor 3's reconnected read-only baseline is healthy: Axis alarm `0x4040:03`
and error `0x703F` are `0x0000`, status `0x7041=0x0270`, raw position is 2,726
counts, A=1/B=1, supported modes are `0x01A5`, and the controller alarm bitmap
is zero. The standalone commissioning path uses 10,000 counts/mm and permits
only startup-relative +/-0.500 mm at 2 mm/s until physical direction and
scaling are confirmed.

The first startup-relative +0.100 mm Motor 3 test moved raw feedback from 2,726
to 3,721 counts. The 995-count change equals 0.0995 mm, and `/joint_states`
settled at 0.0003721 m with alarm `0x0000`. Address and 10,000-count/mm scaling
are therefore validated over this tiny positive move; physical direction and
the reverse return still require operator confirmation.

The subsequent startup-relative zero return settled at 2,730 counts, four
counts (0.0004 mm) from the 2,726-count origin, with alarm `0x0000`. The first
tiny round trip is electrically repeatable; physical direction and smoothness
remain pending operator confirmation.

The operator confirmed positive counts move Motor 3 in the required forward
direction and that both directions of the 0.100 mm round trip were smooth.
Motor 3 direction and first-stage physical behavior are validated.

The subsequent +0.500 mm round trip reached 4,999 counts (0.4999 mm) forward
and returned within two counts (0.0002 mm) of start, with alarm `0x0000`. The
next standalone envelope is +/-2.000 mm at the required 8 mm/s and a symmetric
50 mm/s^2 ramp so the trajectory can actually reach 8 mm/s.

The +2.000 mm requirement-speed test settled 19,999 counts (1.9999 mm) above
start and returned within two counts (0.0002 mm), with alarm `0x0000`. Motor 3
scaling and repeatability are validated with the configured 8 mm/s and
50 mm/s^2 motion profile; physical smoothness at this speed awaits operator
confirmation.

The operator confirmed that the 2 mm requirement-speed round trip was smooth.
Motor 3 standalone commissioning is complete at 8 mm/s and 50 mm/s^2 with no
alarm. The CiA-402 plugin now includes a tertiary state machine at object offset
`0x1000`; the combined backend and GUI map it through the custom
`motor3_position` interface. Software integration is complete, while the first
combined physical test also passed: 0.2732 mm to 0.3727 mm, followed by an
alarm-free return to absolute zero. The operator confirmed smooth motion in the
required forward direction.

## Machine sequence sources

- `sequence.png` shows the mechanical locations and logical Motor 1-6 roles.
- `YuzuSequence.pdf` contains the two-page peeling-operation sequence.
- SHA-256 `sequence.png`:
  `fe20d4c952a1d24efae66022b6038ab28eeffe6b267f091a7db9050905e603b4`
- SHA-256 `YuzuSequence.pdf`:
  `05f40630545ee87e0b9fa3f5fe832f7ff54d79560de0b1faa498347749dd29ec`

The PDF specifies:

| Logical motor | Machine role / sequence action |
| --- | --- |
| Motor 1 | Yuzu holding set; approach for placement, later HOME |
| Motor 2 | Yuzu rotation, 200-250 rpm; CCW during rotation/feed |
| Motors 3 and 4 | Peeling-depth feed; FD for grip/feed and HOME for release |
| Motor 5 | Peeler positioning `90° ROT`, then CW rotation at 20 rpm |
| Motor 6 | Conveyor feed / yuzu positioning FD |

The PDF does not explicitly draw Motor 5's 90-degree CCW return. The operator
has separately confirmed that the required indexed action is 90 degrees CW and
then 90 degrees CCW back to the captured starting position. Treat that return
as an additional confirmed requirement pending a revised sequence drawing.

## Axis 1: Motor 1 linear slide

Current hardware is an `AZM46AK` motor with a label stating that its parameters
are set for `EZSM3LD040AZAK`. The actuator has a 12 mm lead, 400 mm stroke,
600 mm/s catalog maximum speed, and no electromagnetic brake. The assembled
machine, not the catalog stroke, determines the usable software limits.

Live reads after rewiring reported no alarm (`0x603F=0x0000`), normal stopped
status (`0x6041=0x0270`), position 1,700 counts, and electronic gear A=1/B=1.
Thus the current conversion is:

```text
10,000 counts/motor revolution
12 mm/revolution
833.333333 counts/mm
833,333.333333 counts/m
0.0012 mm/count
0.0000012 m/count
```

An initial stopped SDO read returned 1,700 counts. The subsequent ROS
feedback-only launch returned exactly 1,924 counts = 2.3088 mm, so that latest
measured value is the commissioning start. Initial motion is restricted to
2.3088..3.3088 mm, moving only away from the observed start end. The first
target was 2,007 counts = 2.4084 mm, a +0.0996 mm move.

The later +0.9996 mm test produced overload `0xFF30`: the actuator advanced
only to about 2,019 counts while the target reached 2,757. Inspection showed
the carriage at the lower physical end, so increasing counts was treated as
the unsafe direction. After alarm reset, the actual position was 1,999 counts
(2.3988 mm). The revised recovery window permits only decreasing counts,
1.3992..2.3988 mm. Its first opposite-direction target is 1,916 counts =
2.2992 mm, a -0.0996 mm move. Do not interpret the drive coordinate as
measured distance from the physical end, and do not write a new home offset
during initial commissioning.

The opposite-direction recovery tests then succeeded. A target of 1,916 counts
settled at 1,917 counts, and a staged target of 1,582 counts settled at 1,586
counts (1.9032 mm). The operator physically confirmed that decreasing counts
move the carriage upward, away from the lower stop. The total measured travel
from the 1,999-count post-reset capture was -413 counts = -0.4956 mm, with no
reported alarm. Increasing counts is therefore downward on the installed
mechanism and must remain blocked at the captured lower-end boundary.

The final staged target in this first window was 1,166 counts = 1.3992 mm.
Feedback settled at 1,172 counts = 1.4064 mm, six counts (0.0072 mm) from the
target. Relative to the 1,999-count post-reset lower-end capture, measured
upward travel was 827 counts = 0.9924 mm. Axis 1 alarm `0x603F` remained
`0x0000`. This completes the first approximately 1 mm upward commissioning
move on the assembled Motor 1 mechanism; it does not yet define a machine home
or authorize downward travel into the lower stop.

For the next commissioning stage, the 1,999-count lower-end capture is used as
a provisional software zero without writing the drive's home offset. ROS Motor
1 coordinates are now positive upward:

```text
ROS command to raw counts: raw = -833333.3333333334 * position_m + 1999
Raw counts to ROS state:    position_m = -0.0000012 * raw + 0.0023988
```

At the last raw feedback of 1,172 counts, ROS therefore reports 0.9924 mm above
the provisional zero. The initial operational envelope is only 0..5 mm upward.
Commissioning speed was increased from 0.5 to 2 mm/s and acceleration from 1
to 5 mm/s^2. These are conservative assembly-test values, not final machine
requirements or actuator capability limits.

The 2 mm/s stage reached 1.9920 mm and held for two minutes while enabled, with
no alarm. A subsequent safe-park command requested 0.0996 mm before shutdown;
the stopped SDO read was raw 1,898 = 0.1212 mm above provisional zero and the
alarm remained `0x0000`. Because zero is the captured lower mechanical end,
the runtime guard now uses 0.0996 mm as its minimum command and rejects an
exact-zero target.

### Previous Axis 1 commissioning hardware (disconnected)

Oriental Motor identifies `DR28T1A03-AZAKR` as:

- 28 mm frame, guided table-type compact electric cylinder
- rolled ball screw
- 1 mm lead
- 30 mm stroke
- right-side cable
- no mounting plate, electromagnetic brake, or ball-screw cover
- maximum speed 40 mm/s
- maximum acceleration 0.2 m/s^2
- maximum horizontal/vertical transported mass 4 kg
- maximum thrust and holding force 40 N
- repetitive positioning accuracy ±0.01 mm
- lost motion 0.05 mm maximum
- minimum travel amount 0.001 mm

The catalog's 0.001 mm minimum travel amount is a mechanical positioning
specification, not the EtherCAT count conversion. The configured resolution is:

```text
counts/motor revolution = 10,000 × Electronic gear B / Electronic gear A
Axis 1 counts/mm = counts/motor revolution / 1 mm lead
```

For example, electronic gear A=1 and B=1 would mean 10,000 counts/mm. The live
values of `0x6091:01` and `0x6091:02` must be read before ROS scaling is set.

Live Axis 1 values:

| Object | Value | Meaning |
| --- | ---: | --- |
| `0x6091:01` | 1 | Electronic gear A |
| `0x6091:02` | 1 | Electronic gear B |
| `0x607D:01` | -2,147,483,648 | Drive minimum software limit effectively disabled |
| `0x607D:02` | 2,147,483,647 | Drive maximum software limit effectively disabled |
| `0x607C:00` | 0 | Home offset |

Therefore the verified Axis 1 conversion is:

```text
10,000 counts/mm
10,000,000 counts/m
0.0001 mm/count
0.0000001 m/count
```

For the ROS driver channel factors, command conversion requires
`10,000,000` counts/m and state conversion requires `0.0000001` m/count.
These factors must be applied in opposite directions on RxPDO and TxPDO.

Because the drive software limits span the entire signed 32-bit range, they do
not protect the physical 30 mm stroke. Conservative ROS limits and physical
position confirmation are mandatory before motion.

Before Axis 1 motion:

1. Confirm the 30 mm mechanism travel and safe direction in the actual assembly.
2. Keep initial software limits inside the physical stroke, with extra margin.
3. Confirm the actuator fixed-value/recovery data has been loaded into the
   AZD3A using MEXE02 as required by the vendor manual.
4. Read the configured electronic gear/mechanism parameters from MEXE02 or SDO.

## Axis 2: rotary Motor 2

The corrected Axis 2 / machine Motor 2 model is `AZM46AK-FC7.2UA`. Its
sequence role is rotational motion at about 200-250 rpm with spin/stop control.
Official product data identifies it as a 7.2:1 right-angle FC geared motor with
a permissible output-speed range of 0 to 416 r/min, 0.7 N m holding torque,
25 arc-minute (0.42 degree) backlash, and no electromagnetic brake.

The live Axis 2 reads on 2026-08-06 were electronic gear A=1 and B=1,
mechanism setting=1, gear-ratio override=0, and rotation-direction setting=1.
HM-60323-7E section 3-2 defines **motor output shaft** resolution as
`10,000 * B / A`. Applying the FC7.2 gearhead gives:

```text
72,000 counts/machine-output revolution
0.00008726646259971647 output rad/count
11459.155902616465 counts/output rad
```

The catalog's 0.05 degree/pulse value is explicitly for its 1000 P/R setting;
it is not substituted for the live EtherCAT electronic-gear calculation.
Objects `0x6864` and `0x686C` are respectively actual position in steps and
actual velocity in Hz (motor counts/s), so the gearbox must be included when
converting them to machine-output radians and radians/second. Vendor object
`0x4067:02` reports feedback speed in r/min and should be used in the repeat
physical validation.

The first ROS stage uses `GenericEcSlave`, Controlword=0, mode=0, no command
interface, RxPDO `0x1610`, and TxPDO `0x1A11`. It is feedback-only and must not
enable Axis 2. Do not reuse the `AZM46AK-FC20DA` 20:1 scaling for Axis 2.

For guarded motion commissioning, operators publish RPM directly to
`/axis2_velocity_controller/commands_rpm`. The guard checks the active RPM
boundary, applies an RPM/s acceleration ramp and command watchdog, converts
RPM to rad/s internally, and alone publishes to the private ros2_control topic
`/axis2_raw_velocity_controller/commands`.

Corrected live commissioning passed at 25, 100, 200, and 250 machine-output
rpm. The present hardware also passed an experimental 250 output-rpm/s ramp,
reaching 250 rpm in about one second. This acceleration is load-dependent and
is not an Oriental Motor rated maximum; repeat the test after installation of
the final driven load before adopting it as a production setting.

## Axis 1 CSP startup from a retained position

Axis 1 may be stopped at any valid guarded position; it does not need to return
to zero before ROS exits. A startup at a retained 14.9993 mm exposed that the
generic position controller can briefly emit zero before the guard receives
feedback, causing AZD3A alarm `0xFF34`. The Axis 1 motion xacro therefore sets
the CiA 402 plugin's `position_startup_tolerance` to `0.00001 m`. CSP output is
held at measured position until the raw controller and feedback agree within
0.01 mm, after which normal guarded commands are accepted.

Because `/joint_states` may briefly contain zero before EtherCAT reaches OP,
the Axis 1 guard does not latch only its first sample. Until the first accepted
operator target, it continuously tracks measured feedback and publishes that
value to the raw controller. Live validation succeeded from a retained
approximately 15 mm position: startup remained fault-free and the subsequent
guarded return command produced physical motion.

## Historical Axis 3 commissioning: rotary Motor 5 (currently disconnected)

Oriental Motor identifies `AZM46AK-FC20DA` as:

- 42 mm AZ Series closed-loop stepper motor
- mechanical absolute encoder
- right-angle spur/face gear
- 20:1 gear ratio
- no electromagnetic brake
- permissible output speed 0 to 150 r/min
- nominal resolution 0.018 degrees/pulse when the resolution setting is
  1000 P/R

The live A=1/B=1 setting is 10,000 counts per motor revolution. Applying the
FC20 gearhead gives the machine-output conversion:

```text
200,000 counts/machine-output revolution
0.00003141592653589793 output rad/count
31830.98861837907 counts/output rad
```

The required machine
sequence is a relative 90-degree clockwise move followed by a 90-degree
counterclockwise return to the captured starting position, at no more than
20 rpm. At 200,000 counts per output revolution, 90 degrees equals 50,000
counts. The physically verified positive ROS direction is clockwise when
viewed from the output-shaft front.

Live corrected-scaling validation moved from `-57.165142491030196` to
`-55.5943461642353` output rad: exactly pi/2 rad. The physical output mark also
moved one quarter-turn CW. Thus the 200,000-count/output-revolution conversion
is verified for position indexing.

Using the same corrected conversion, staged 5 and 10 output-rpm tests and the
final 20 output-rpm CW test completed successfully. This validates both Motor
5 actions shown in the sequence PDF: 90-degree positioning and subsequent
20 rpm clockwise rotation.

## Axis-specific CiA 402 objects

| Function | Axis 1 | Axis 2 | Axis 3 |
| --- | --- | --- | --- |
| Controlword | `0x6040` | `0x6840` | `0x7040` |
| Statusword | `0x6041` | `0x6841` | `0x7041` |
| Mode command | `0x6060` | `0x6860` | `0x7060` |
| Mode display | `0x6061` | `0x6861` | `0x7061` |
| Actual position | `0x6064` | `0x6864` | `0x7064` |
| Target position | `0x607A` | `0x687A` | `0x707A` |
| Default RxPDO 1 | `0x1600` | `0x1610` | `0x1620` |
| Default TxPDO 1 | `0x1A00` | `0x1A10` | `0x1A20` |

The first ROS experiment remains Axis 1 only. Axes 2 and 3 should not be added
until Axis 1 communication, stopping, scaling, and limits are repeatable.

## Read-only baseline

With the drive in PREOP and no motion mode selected:

| Axis | Error code | Statusword | Mode display | Actual position |
| --- | --- | --- | --- | --- |
| Axis 1 | `0x0000` | `0x0270` | `0` | `-16` |
| Axis 2 | `0x0000` | `0x0270` | `0` | `5,641,556` |
| Axis 3 | `0x0000` | `0x0270` | `0` | `-1,970,446` |

All axes reported no error and the safe switch-on-disabled state. The Axis 2/3
values are valid absolute multi-turn position counts, not fault codes.

## Official references

- AZD3A-KED:
  <https://catalog.orientalmotor.com/item/az-series-multi-axis-controllers-drivers/ethercat-multi-axis-controllers-az-dc-input/azd3a-ked>
- DR28T1A03-AZAKR:
  <https://www.orientalmotor.co.jp/ja/products/detail?hinmei=DR28T1A03-AZAKR>
- AZM46AK-FC7.2UA:
  <https://catalog.orientalmotor.com/item/az-series-42mm-absolute-stepper-motors/az-series-42mm-absolute-encoder-stepper-motors-dc/azm46ak-fc7-2ua>
- AZ Series family catalog:
  `vendor/oriental_motor/AZ_Family_Catalog_2018-2019.pdf`
- AZM46AK-FC20DA:
  <https://catalog.orientalmotor.com/item/az-series-42mm-absolute-stepper-motors/az-series-42mm-absolute-encoder-stepper-motors-dc/azm46ak-fc20da>
- AZM46AK-PS50 Motor 6 product page (specifications checked 2026-09-21):
  <https://catalog.orientalmotor.com/item/42mm-frame-stepper-motors/az-series-42mm-absolute-encoder-stepper-motors-dc/azm46ak-ps50>
- MISUMI SVKA catalog page, including the 30 mm pulley and ordering fields
  (specifications checked 2026-09-21):
  <https://us.misumi-ec.com/pdf/fa/2019/2019_US_1254.pdf>
  Local archived copy:
  `vendor/misumi/SVKA_catalog_2019_pages_1254-1255.pdf` (SHA-256
  `180f1ce5489386ba3484f3c16d8b2449b20baf5ebc22bc15fa62572931a82199`)
- MISUMI conveyor belt calculation table, independently listing the SVKA
  30 mm pulley (checked 2026-09-21):
  <https://us.misumi-ec.com/maker/misumi/mech/product/cvs/calculation/>
- Multi-axis EtherCAT manual:
  `vendor/oriental_motor/HM-60323-7E.pdf`
