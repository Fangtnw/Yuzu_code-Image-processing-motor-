# Installed motor speed ratings and project caps

Checked against Oriental Motor's official model-specific catalog pages on
2026-10-01. These links are the source of record for the published speed
ratings below; use the model number on the hardware label when opening them.

| Motor | Installed model | Catalog maximum (output speed for geared motors) | Project cap (80%) | Official source |
|---|---|---:|---:|---|
| 1 | EZSM3LD040AZAK | 600 mm/s | 480 mm/s | [EZS3R slide, exact model](https://catalog.orientalmotor.com/en-us/item/ezs-reversed-motor-type-linear-slides-az-dc/ezs3r-reversed-motor-type-linear-slides-az-dc/ezsm3ld040azak) |
| 2 | AZM46AK-FC7.2UA, 7.2:1 | 416 rpm | 332.8 rpm | [AZM46AK-FC7.2UA](https://catalog.orientalmotor.com/item/az-series-42mm-absolute-stepper-motors/az-series-42mm-absolute-encoder-stepper-motors-dc/azm46ak-fc7-2ua) |
| 3 | DR28T1A03-AZAKR | 40 mm/s | 32 mm/s | [DR28T1A03-AZAKR product family/configuration](https://catalog.orientalmotor.com/item/table-type-compact-electric-cylinders-dr-drs-az/dr28t-compact-electric-cylinders-az-series/dr28t1a03-azakr-p) |
| 4 | DR28T1A03-AZAKR | 40 mm/s | 32 mm/s | [DR28T1A03-AZAKR product family/configuration](https://catalog.orientalmotor.com/item/table-type-compact-electric-cylinders-dr-drs-az/dr28t-compact-electric-cylinders-az-series/dr28t1a03-azakr-p) |
| 5 | AZM46AK-FC20DA, 20:1 | 150 rpm | 120 rpm | [AZM46AK-FC20DA](https://catalog.orientalmotor.com/item/az-series-42mm-absolute-stepper-motors/az-series-42mm-absolute-encoder-stepper-motors-dc/azm46ak-fc20da) |
| 6 | AZM46AK-PS50, 50:1 | 60 rpm | 48 rpm | [AZM46AK-PS50](https://catalog.orientalmotor.com/item/ll-categories-shop-online-stepper-motor-components/az-stepper-motors-absolute-encoders-dc-cable-type/azm46ak-ps50) |

The DR28 actuator manual also documents the DR28 family motion ratings; the
official PDF is [HL-14106-6E](https://www.orientalmotor.com/products/pdfs/opmanuals/HL-14106-6E.pdf).
The exact project actuator is `DR28T1A03-AZAKR`; the catalog link above is the
same base actuator with a foot-mount suffix (`-P`).

## Software acceleration/deceleration profiles

Motor 1 uses a 1-second ramp after abrupt motion was reported. Motors 2–6 use
0.5-second ramps. For constant-acceleration ramps, `acceleration = operating
speed / ramp time`. The same magnitude is used for deceleration. These are
command-profile settings, not claims of manufacturer-rated acceleration.

| Motor | 80% speed cap | Command ramp | Basis / qualification |
|---|---:|---:|---|
| 1 | 480 mm/s | 480 mm/s² | 1 s software profile; the exact actuator catalog page gives speed and stroke, but no acceleration rating. |
| 2 | 332.8 rpm | 665.6 rpm/s | Software profile; the rotary catalog gives permissible speed, not maximum acceleration. |
| 3 | 32 mm/s | 64 mm/s² | 0.5 s profile; published DR28 max acceleration is 200 mm/s², so this is below that maximum. |
| 4 | 32 mm/s | 64 mm/s² | Same DR28 specification and profile as Motor 3. |
| 5 | 120 rpm | 240 rpm/s | Software profile; the rotary catalog gives permissible speed, not maximum acceleration. |
| 6 | 48 rpm | 96 rpm/s | Software profile; the rotary catalog gives permissible speed, not maximum acceleration. |

## Interpretation and operating cautions

- Catalog permissible/max speed is an individual product rating, not a
  guarantee that the assembled peeler can sustain that speed under its load.
- The project application ranges previously noted for Motor 2 (250–300 rpm)
  and Motor 5 (90–150 rpm) are not the catalog's maximum-permissible ranges
  for these exact models. The catalog
  lists 0–416 rpm and 0–150 rpm, respectively. If those lower values are
  intended machine-level ceilings, the 80% caps must be recalculated from
  those application limits instead.
- Motors 2–5 operate simultaneously in the machine sequence. Model-specific
  catalog maxima do not certify the shared power supply, thermal duty, torque,
  or mechanism for simultaneous maximum-speed operation; commission the
  combined sequence with staged, measured tests.
- A positioning move must have enough travel to accelerate and brake. A short
  move may begin decelerating before it reaches the configured speed cap, even
  though the acceleration setting would reach that cap in its configured ramp
  time on a long enough move. For example, Motor 6's 30 mm step is shorter than the distance
  needed to accelerate to 48 rpm and stop with the 96 rpm/s profile. At Motor
  1's 480 mm/s cap and 480 mm/s² ramp, a rest-to-rest move needs about 480 mm
  to accelerate and brake, exceeding its 400 mm guarded stroke.
- The Motor 1 profile requires 0.48 m/s² on a loaded vertical assembly. No
  matching maximum acceleration is stated on the catalog page; validate at
  low speed and with the mechanism secured before increasing speed.

## Existing local vendor documents

- `HM-60323-7E.pdf`: AZD3A-KED EtherCAT driver manual (stored beside this file).
- `AZ_Family_Catalog_2018-2019.pdf`: AZ-series family catalog (stored beside
  this file). It is supplemental; use the current model-specific product pages
  above for the speed limits.
- `../misumi/SVKA_catalog_2019_pages_1254-1255.pdf`: conveyor source for the
  SVKA conveyor/pulley dimensions; see `vendor/misumi/README.md`.
