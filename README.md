# Yuzu Peeler Motor Control

ROS 2 Humble application for the six-motor Yuzu peeler prototype. The project
uses `ros2_control`, the ICube EtherCAT driver, CiA-402 drives, and an IgH
EtherCAT master. The operator GUI provides guarded manual control and an
operator-advanced peeling sequence.

## Scope and safety

This repository contains control software for laboratory commissioning. It is
not a substitute for a machine safety system. Keep the physical emergency
power cutoff accessible, test with the mechanism clear, and run only one
EtherCAT ROS launch at a time.

Motion is intentionally locked until the GUI sees fresh feedback and all six
drives report CiA-402 Operation Enabled (`0x0567` pattern).

## Repository layout

- `motion_cmd/` — ROS 2 `motor_controller` package, guards, GUI, launch files,
  EtherCAT mappings, URDF/Xacro, and operator documentation.
- `scripts/` — reproducible workspace setup and startup helpers.
- `tests/` — offline configuration and command tests; no motors are required.
- `vendor/` — supplied Oriental Motor and MISUMI reference documents.
- `progress.md` — dated engineering record and commissioning history.
- `UBUNTU_ETHERCAT_SETUP_GUIDE.md` — kernel/module/NIC setup for a new Ubuntu PC.

## Supported deployment

The validated environment is Ubuntu 22.04, ROS 2 Humble, Python 3.10, and a
dedicated wired Ethernet NIC connected to EtherCAT. The IgH kernel modules
must be built for the running kernel; they cannot be copied between kernels.
Read the EtherCAT setup guide before connecting power.

## New-PC setup

1. Install Ubuntu 22.04 and ROS 2 Humble.
2. Install/build the IgH EtherCAT master and configure the customer's NIC as
   described in [`UBUNTU_ETHERCAT_SETUP_GUIDE.md`](UBUNTU_ETHERCAT_SETUP_GUIDE.md).
3. Clone this repository and run:

   ```bash
   cd Yuzu_code-Image-processing-motor-
   ./scripts/setup_workspace.sh --workspace "$HOME/kyutech/azd3a_ws"
   ```

   The script imports the pinned external ROS dependency, links this package
   into the workspace, installs rosdep dependencies, and performs a Release
   build. It does not modify EtherCAT kernel modules or system NIC settings.

4. Configure `/etc/sysconfig/ethercat`, udev permissions, and the `ethercat`
   group for the actual PC. Then start the system with:

   ```bash
   ./scripts/start_yuzu.sh --workspace "$HOME/kyutech/azd3a_ws"
   ```

   The start helper sources ROS and launches the integrated GUI; it does not
   silently restart EtherCAT or power hardware.

## Manual build and launch

```bash
source /opt/ros/humble/setup.bash
cd "$HOME/kyutech/azd3a_ws"
colcon build --packages-select motor_controller --symlink-install
source install/setup.bash
ros2 launch motor_controller azd3a_motor1_motor2_gui.launch.py
```

Verify the EtherCAT link first:

```bash
/opt/etherlab/bin/ethercat master
/opt/etherlab/bin/ethercat slaves
ros2 control list_controllers
```

## Tests

The offline test suite validates mappings, launch files, guards, and GUI
configuration:

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
python3 -m py_compile motion_cmd/motor_controller/*.py motion_cmd/launch/*.py
```

Physical motion tests must be performed on the designated hardware and are
documented separately; never replace them with offline tests.

## Current integrated axes

| Motor | Function | GUI command units | Project operating limit |
|---|---|---|---|
| 1 | vertical Yuzu placement | mm | 0–400 mm, 15 mm/s |
| 2 | Yuzu rotation | rpm | ±250 rpm, up to 500 rpm/s ramp |
| 3 | gripping/feed slide | mm | 0–20 mm, 8 mm/s |
| 4 | peeling-depth slide | mm | 0–20 mm, 8 mm/s |
| 5 | peeler index | degrees from runtime zero | ±180°, 20 rpm |
| 6 | conveyor | mm/s and relative distance steps | 0–90 mm/s GUI speed, 30 mm default step |

These are project guards, not universal manufacturer limits. Confirm the
hardware envelope in `motion_cmd/AZD3A_HARDWARE.md` and the GUI details panel.
