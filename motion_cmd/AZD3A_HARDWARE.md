# AZD3A-KED Connected Hardware

This records the physical devices connected during the one-drive bring-up.
Model strings should be checked against the physical labels before scaling or
motion limits are treated as final.

Axis numbering here follows the current AZD3A bring-up wiring. It is not the
same thing as the final machine motor numbering. Axis 1 currently uses a
`DR28T1A03-AZAKR` linear actuator for EtherCAT/ROS 2 validation. That actuator
model is the machine Motor 3/4 type in the mechanism plan, but it is wired to
AZD3A Axis 1 right now.

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
| Axis 1 | `DR28T1A03-AZAKR` | Linear, machine Motor 3/4 type currently wired to Axis 1 | Confirmed in Oriental Motor catalog |
| Axis 2 | `AZM46AK-FC7.2UA` | Rotary speed / spin-stop, machine Motor 2 | User-corrected physical model; verify scaling before motion |
| Axis 3 | `AZM46AK-FC20DA` | Indexed rotary position, machine Motor 5: +90 degrees CW then -90 degrees CCW at up to 20 rpm | Confirmed model; sequence clarified by operator |

These are temporary bring-up ports on the currently available AZD3A-KED:

| Current local port | Temporary connected machine role |
| --- | --- |
| AZD3A slave 0, Axis 1 | Motor 3/4 actuator type (one `DR28T1A03-AZAKR`) |
| AZD3A slave 0, Axis 2 | Motor 2 (`AZM46AK-FC7.2UA`) |
| AZD3A slave 0, Axis 3 | Motor 5 (`AZM46AK-FC20DA`) |

This table is commissioning evidence, not the final harness assignment. When
the remaining motors and second AZD3A-KED arrive, record an explicit mapping
for each logical `motor_1` through `motor_6` to `{slave alias, local axis}`.
Prefer persistent EtherCAT aliases over chain position if the installation
allows it, because physical slave position can change when cabling changes.

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

## Axis 1: linear actuator

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

## Axis 3: rotary Motor 5

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
- Multi-axis EtherCAT manual:
  `vendor/oriental_motor/HM-60323-7E.pdf`
