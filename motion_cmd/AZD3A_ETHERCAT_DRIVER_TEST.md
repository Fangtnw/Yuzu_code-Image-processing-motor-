# AZD3A-KED Basic EtherCAT Test

This is the first test for your Person 2 role: prove the Ubuntu EtherCAT stack
can see one AZD3A-KED and read its PDO/SDO map.

Keep this test small. Do not involve URDF, robot description, controller manager,
or multi-axis motion yet.

## Files Kept For This Test

- `config/azd3a_ked_cia402_slave.yaml`
- `motor_controller/azd3a_ethercat_check.py`

The YAML records the AZD-KED ESI identity and important CiA402 objects:

- vendor: `0x000002BE`
- product: `0x000013AF`
- revision: `0x01110301`
- RxPDO: `0x1600`
- TxPDO: `0x1A00`
- objects: `0x6040`, `0x6041`, `0x6060`, `0x6061`, `0x6064`, `0x607A`

## Hardware Setup

Connect one AZD3A-KED only.

- Ubuntu spare Ethernet port to AZD3A-KED `ECAT IN`
- 24/48 VDC main power
- 24 VDC control power
- Proper grounding
- Motor mechanically safe

## IgH EtherCAT Test

Run:

```bash
sudo /etc/init.d/ethercat start
ethercat slaves
ethercat pdos
ethercat sdos
```

Expected:

- `ethercat slaves` lists one `AZD-KED`.
- Slave position is usually `0:0`.
- `ethercat pdos` includes the CiA402 objects listed above.
- `ethercat sdos` can read the object dictionary without communication errors.

## Package Surface Check

After building and sourcing your ROS2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select motor_controller
source install/setup.bash
ros2 run motor_controller azd3a_ethercat_check
```

This Python check controller checks:

- `ethercat slaves`
- `ethercat pdos`
- `ethercat sdos`
- `ethercat_driver` package availability
- `ethercat_generic_slave` package availability
- `ethercat_generic_cia402_drive` package availability

To see what it will run without touching hardware:

```bash
ros2 run motor_controller azd3a_ethercat_check --dry-run
```

## Pass Condition

The basic EtherCAT layer is ready when:

1. IgH sees exactly one AZD-KED.
2. PDOs match the AZD-KED ESI objects.
3. SDO reads work.
4. `ethercat_driver_ros2` packages are installed and visible to ROS2.

After this passes, the next step is a separate minimal driver-node experiment.
That step can be added later without keeping robot-description files in this
workspace now.
