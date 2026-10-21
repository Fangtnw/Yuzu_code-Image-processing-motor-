# Yuzu Peeler hardware configuration

The integrated GUI uses two AZD3A-KED controllers and six motors. The table
records the supplied software mapping; confirm physical labels, wiring order,
electronic gearing and position references before commissioning a new machine.

| Motor | EtherCAT slave position | Local axis | Configured model / function |
|---|---:|---:|---|
| 1 | 0 | 1 | EZSM3LD040AZAK vertical slide |
| 2 | 0 | 2 | AZM46AK-FC7.2UA rotation |
| 3 | 0 | 3 | DR28T1A03-AZAKR gripping/feed slide |
| 4 | 1 | 1 | DR28T1A03-AZAKR peeling-depth slide |
| 5 | 1 | 2 | AZM46AK-FC20DA peeler index |
| 6 | 1 | 3 | AZM46AK-PS50 conveyor |

The supplied drive identity is vendor `0x000002BE`, product `0x000013AF`,
revision `0x01110301`. The configuration addresses the drives by chain position;
changing cable order changes which controller is addressed.

The integrated configuration is defined by:

- `urdf/azd3a_motor1_motor2.urdf.xacro`
- `config/azd3a_motor1_motor2_slave.yaml` (Motors 1–3)
- `config/azd3a_motor4_motor6_slave.yaml` (Motors 4–6)
- `config/azd3a_motor1_motor2_controllers.yaml`

The filenames are retained for compatibility and cover all six motors.
Motor 1 includes a direction reversal and encoder offset in its PDO mapping;
these are machine calibration values, not generic defaults. Motors 3/4 use
10,000 counts/mm. Verify the supplied calibration against the installed machine.
Do not alter scaling or offsets as part of ordinary startup.

See [Operator GUI](YUZU_OPERATOR_GUI.md) for ranges, units and validation status.
Manufacturer reference material is maintained separately from this operator bundle. Obtain the applicable model documentation when verifying the installed hardware.
