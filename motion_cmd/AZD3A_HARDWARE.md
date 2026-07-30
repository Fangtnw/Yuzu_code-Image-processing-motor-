# AZD3A-KED Connected Hardware

This records the physical devices connected during the one-drive bring-up.
Model strings should be checked against the physical labels before scaling or
motion limits are treated as final.

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
| Axis 1 | `DR28T1A03-AZAKR` | Linear | Confirmed in Oriental Motor catalog |
| Axis 2 | `AZM46AK-FC20DA` | Rotary | Confirmed in Oriental Motor catalog |
| Axis 3 | `AZM46AK-FC20DA` | Rotary | Confirmed in Oriental Motor catalog |

## Axis 1: linear actuator

Oriental Motor identifies `DR28T1A03-AZAKR` as:

- 28 mm frame, guided table-type compact electric cylinder
- rolled ball screw
- 1 mm lead
- 30 mm stroke
- right-side cable
- no mounting plate, electromagnetic brake, or ball-screw cover
- maximum speed 40 mm/s
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

## Axes 2 and 3: geared rotary motors

Oriental Motor identifies `AZM46AK-FC20DA` as:

- 42 mm AZ Series closed-loop stepper motor
- mechanical absolute encoder
- right-angle spur/face gear
- 20:1 gear ratio
- no electromagnetic brake
- permissible output speed 0 to 150 r/min
- nominal resolution 0.018 degrees/pulse when the resolution setting is
  1000 P/R

At that stated setting, the output-shaft conversion is:

```text
20,000 counts/output revolution
0.000314159265 rad/count
3183.098862 counts/rad
```

These values are reference calculations, not yet active configuration.
MEXE02/SDO settings must confirm that the actual electronic gear and resolution
match the catalog condition before they are used as ROS scaling factors.

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
- AZM46AK-FC20DA:
  <https://catalog.orientalmotor.com/item/az-series-42mm-absolute-stepper-motors/az-series-42mm-absolute-encoder-stepper-motors-dc/azm46ak-fc20da>
- Multi-axis EtherCAT manual:
  `vendor/oriental_motor/HM-60323-7E.pdf`
