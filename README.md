# Yuzu Peeler Motor Control

ROS 2 Humble application for the six-motor Yuzu peeler prototype. The project
uses `ros2_control`, the ICube EtherCAT driver, CiA-402 drives, and an IgH
EtherCAT master. The operator GUI provides guarded manual control and an
operator-configurable peeling sequence.

## Scope and safety

This source bundle contains control software for laboratory commissioning. It is
not a substitute for a machine safety system. Keep the physical emergency
power cutoff accessible, test with the mechanism clear, and run only one
EtherCAT ROS launch at a time.

Motion is intentionally locked until the GUI sees fresh feedback and all six
drives report CiA-402 Operation Enabled (`(statusword & 0x006F) == 0x0027`).

## Bundle layout

- `motion_cmd/` — ROS 2 `motor_controller` package, guards, GUI, launch files,
  EtherCAT mappings, URDF/Xacro, and operator documentation.
- `scripts/` — reproducible workspace setup and startup helpers.
- `tests/` — offline configuration and command tests; no motors are required.
- `vendor/` — supplied Oriental Motor and MISUMI reference documents.
- `patches/` — versioned extension required by the multi-axis EtherCAT driver.
- `LICENSE` and `THIRD_PARTY_NOTICES.md` — licensing and reference-document scope.
- `UBUNTU_ETHERCAT_SETUP_GUIDE.md` — kernel/module/NIC setup for a new Ubuntu PC.

## Supported deployment

The development environment is Ubuntu 22.04, ROS 2 Humble, Python 3.10, and a
dedicated wired Ethernet NIC connected to EtherCAT. The IgH kernel modules
must be built for the running kernel; they cannot be copied between kernels.
Read the EtherCAT setup guide before connecting power.
The supplied bundle has offline checks; a complete installation and machine
acceptance test must still be performed on the recipient's PC.

## Start here: supplied ZIP

Extract the supplied ZIP to a permanent location. Open a terminal in the
extracted folder containing this README, `scripts/`, and `motion_cmd/`.
No checkout of the Yuzu project is required.

Keep this folder in place after setup: the workspace links to its source files.
Do not build from a temporary archive-preview folder or delete the extracted
folder after installation. The ZIP contains application source, driver patches,
tests and reference documents, not a preinstalled ROS/EtherCAT system.

For source review only, start with
[`motion_cmd/YUZU_OPERATOR_GUI.md`](motion_cmd/YUZU_OPERATOR_GUI.md),
[`motion_cmd/launch/yuzu_peeler.launch.py`](motion_cmd/launch/yuzu_peeler.launch.py)
and [`patches/README.md`](patches/README.md). No powered hardware is needed.

## First-time setup on a new PC

1. Follow [`UBUNTU_ETHERCAT_SETUP_GUIDE.md`](UBUNTU_ETHERCAT_SETUP_GUIDE.md)
   to install ROS 2 Humble, Git, vcstool, rosdep, colcon and the IgH EtherCAT
   master on Ubuntu 22.04. Initialize/update rosdep, build kernel modules for
   the running kernel, and configure the actual NIC, permissions and
   `/usr/local/etherlab` compatibility link as described there.
2. From the extracted folder containing this README, run:

   ```bash
   bash scripts/setup_workspace.sh --workspace "$HOME/yuzu_ws"
   ```

   Internet access is required: the script downloads the pinned external
   EtherCAT ROS driver, applies the supplied multi-axis patch, installs rosdep
   dependencies and builds the application and driver dependencies. System
   dependency installation may request administrator privileges. The script
   does not install kernel modules or configure the NIC.
   `$HOME/yuzu_ws` is an example workspace location; use the same absolute
   workspace path in all later commands.

## Daily startup after setup

With the mechanism clear and the physical emergency stop accessible, start
EtherCAT and check that both controllers are detected:

```bash
sudo /etc/init.d/ethercat start
/opt/etherlab/bin/ethercat master
/opt/etherlab/bin/ethercat slaves
```

Then, from the extracted folder containing this README:

```bash
bash scripts/start_yuzu_peeler.sh --workspace "$HOME/yuzu_ws"
```

The helper sources ROS and the workspace and launches the GUI. It does not
start EtherCAT or power hardware. Wait for fresh feedback and the ready
indication before commanding motion. Never restart EtherCAT while a control
session is running.

## Rebuild after source changes

This is for an already configured workspace, not a replacement for first-time
setup. Stop the control session safely before rebuilding.

```bash
source /opt/ros/humble/setup.bash
cd "$HOME/yuzu_ws"
colcon build --packages-up-to motor_controller --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch motor_controller yuzu_peeler.launch.py
```

After launching, inspect controllers from a second terminal:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/yuzu_ws/install/setup.bash"
ros2 control list_controllers
```

## Tests

From the extracted folder containing this README, run the offline tests
(Python dependencies must be installed first). These check configuration and
selected code contracts; they do not certify physical motion:

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
python3 -m py_compile motion_cmd/motor_controller/*.py motion_cmd/launch/*.py
```

Physical motion tests must be performed on the designated hardware and are
documented separately; never replace them with offline tests.

The exported bundle includes `SHA256SUMS`. To verify its files before building:

```bash
sha256sum -c SHA256SUMS
```

## Current integrated axes

The integrated interface uses one consistent public naming convention:
`motor1` through `motor6` for topics, controller names, and guard executables.
The old `axis1`/`axis2` command topics and executable names remain as
compatibility aliases for existing commissioning scripts; new integrations
should use the `motor*` names.

| Motor | Function | GUI command units | Project operating limit |
|---|---|---|---|
| 1 | vertical Yuzu placement | mm | 0–400 mm, 480 mm/s (80% of 600 mm/s actuator maximum) |
| 2 | Yuzu rotation | rpm | ±332.8 rpm (80% of 416 rpm output maximum) |
| 3 | gripping/feed slide | mm | 0–20 mm, 32 mm/s (80% of 40 mm/s actuator maximum) |
| 4 | peeling-depth slide | mm | 0–20 mm, 32 mm/s (80% of 40 mm/s actuator maximum) |
| 5 | peeler index | degrees from runtime zero | ±180°, 120 rpm (80% of 150 rpm output maximum) |
| 6 | conveyor | mm/s and indexed distance steps | 75.4 mm/s (48 rpm, 80% of 60 rpm maximum), 140.825 mm per step |

Motor 1 ramps to/from its 480 mm/s cap in 1 s using 480 mm/s². Motors 2–6 use
0.5 s profiles: Motor 2 665.6 rpm/s; Motors 3/4 64 mm/s²; Motor 5 240 rpm/s;
Motor 6 96 rpm/s. Motors 3/4 remain below their published 200 mm/s²
acceleration maximum. These are software profiles; Motor 1's ramp must be
validated on the loaded vertical assembly. Short position moves can brake
before attaining the speed cap. A Motor 1 rest-to-rest move needs about 480 mm
to accelerate to 480 mm/s and brake at this ramp, so the guarded 400 mm stroke
cannot reach that cap and stop in-range from rest. The manufacturer source register is
`vendor/oriental_motor/MOTOR_SPEED_SOURCES.md`.

Motor 6 steps are controlled in the 200 Hz backend, using absolute encoder
targets from the first step's stopped position. Completion requires a stopped
encoder error within 0.02 mm; this is a software threshold, not a measured
belt accuracy specification. Stops retain the target for **Resume interrupted
step**. Manual running or a backend restart starts a new reference grid.
See [`motion_cmd/MOTOR6_INDEXING.md`](motion_cmd/MOTOR6_INDEXING.md) for the
140.825 mm requirement, test evidence and physical validation still needed.
