# AZD3A-KED EtherCAT on Ubuntu: Setup and Operator Guide

This is the reproducible reference for bringing up one AZD3A-KED on a new
Ubuntu computer. It is also organized so its sections can be reused in progress
presentation slides.

## 1. How the System Works

```text
ROS 2 application / ros2_control
              |
              v
ICube ethercat_driver_ros2 + CiA402 plugin
              |
              v
IgH EtherCAT user library (libethercat)
              |
              v
IgH kernel modules (ec_master + ec_generic)
              |
              v
Dedicated Ubuntu Ethernet NIC
              |
              v
AZD3A-KED ECAT IN -> motor
```

The Ethernet port is dedicated to EtherCAT. EtherCAT does not use ordinary IP
communication, so the motor does not need an IP address. IgH takes ownership of
the selected NIC and exchanges deterministic Ethernet frames with the drive.

The layers have separate jobs:

- **AZD3A-KED:** executes the drive state machine and motor control.
- **IgH master:** discovers slaves and exchanges EtherCAT frames.
- **CiA 402 plugin:** maps standard drive objects such as Controlword,
  Statusword, target position, and actual position.
- **`ros2_control`:** exposes hardware state/command interfaces to ROS
  controllers.
- **This repository:** owns the AZD3A configuration, safety checks, launch code,
  and application-level commands.

## 2. What Belongs Where

### Keep in the main Git repository

Commit these because they are project knowledge:

- `progress.md`
- this setup guide
- `azd3a_ws.repos`
- `motion_cmd/config/azd3a_ked_cia402_slave.yaml`
- ROS launch/URDF/controller configuration when created
- source code and tests

### Keep only on the Ubuntu machine

Do not copy or commit:

- `~/kyutech/azd3a_ws/build/`
- `~/kyutech/azd3a_ws/install/`
- `~/kyutech/azd3a_ws/log/`
- compiled kernel modules
- `/dev/EtherCAT0`

These are generated or machine-specific.

System configuration also remains outside Git:

- `/etc/sysconfig/ethercat`
- `/etc/udev/rules.d/99-EtherCAT.rules`
- `/opt/etherlab`
- `/usr/local/etherlab` compatibility link
- installed kernel modules under `/lib/modules/$(uname -r)/`

The `.md`, `.repos`, source, YAML, and test files let another Codex session
understand and reconstruct the Ubuntu environment. The generated workspace is
not required for code review.

## 3. Hardware and Safety

Required:

- native Ubuntu PC with a dedicated wired Ethernet port
- AZD3A-KED control and main power
- Ethernet cable from the dedicated NIC to `ECAT IN`
- motor and correct motor cable
- proper grounding
- unloaded or mechanically secured motor for first tests
- accessible physical power cutoff

Do not use Wi-Fi or the normal office-network port for EtherCAT. Do not connect
the EtherCAT chain to an ordinary network switch unless it is explicitly an
EtherCAT device.

## 4. New Ubuntu PC Prerequisites

The tested combination is Ubuntu 22.04 and ROS 2 Humble.

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install git build-essential autoconf libtool pkg-config \
  linux-headers-$(uname -r) python3-colcon-common-extensions \
  python3-rosdep python3-vcstool mokutil
```

Install ROS 2 Humble first if `/opt/ros/humble` does not exist. Use the official
ROS 2 Ubuntu installation instructions rather than copying another computer's
`/opt/ros` directory.

## 5. Build and Install IgH EtherCAT

Clone the pinned source used for this bring-up:

```bash
mkdir -p ~/kyutech
cd ~/kyutech
git clone https://gitlab.com/etherlab.org/ethercat.git
cd ethercat
git checkout 6e60da92cd0bffd31d1207f76471d904eff4c2de
./bootstrap
```

Configure against the current kernel:

```bash
./configure \
  --prefix=/opt/etherlab \
  --sysconfdir=/etc \
  --with-linux-dir=/usr/src/linux-headers-$(uname -r) \
  --enable-generic
make all modules
sudo make install modules_install
sudo depmod
```

Confirm:

```bash
modinfo ec_master
modinfo ec_generic
```

Kernel updates require rebuilding and reinstalling these modules for the new
`uname -r`.

## 6. Secure Boot

Unsigned third-party modules may fail with:

```text
Key was rejected by service
```

Preferred production solution: sign `ec_master.ko` and `ec_generic.ko` using an
enrolled Machine Owner Key.

Fast lab solution:

```bash
sudo mokutil --disable-validation
sudo reboot
```

Complete the blue MOK screen flow during boot, then check:

```bash
mokutil --sb-state
```

The tested system reported that Secure Boot remained enabled while validation
was disabled in shim. This is less secure than properly signing the modules and
should be recorded as a lab-machine choice.

## 7. Select and Configure the EtherCAT NIC

List interfaces:

```bash
ip -brief link
ip link
```

Choose the dedicated wired interface. Example for this PC: `eno2`.

Edit `/etc/sysconfig/ethercat`:

```bash
MASTER0_DEVICE="eno2"
DEVICE_MODULES="generic"
UPDOWN_INTERFACES="eno2"
```

Interface names and MAC addresses can differ on every PC. Never blindly copy
`eno2` or `a0:36:bc:31:3f:18` to a new machine.

## 8. Allow ROS to Access the Master Without Root

```bash
sudo groupadd -f ethercat
sudo usermod -aG ethercat "$USER"
echo 'KERNEL=="EtherCAT[0-9]*", GROUP="ethercat", MODE="0660"' | \
  sudo tee /etc/udev/rules.d/99-EtherCAT.rules
sudo udevadm control --reload-rules
sudo reboot
```

After reboot:

```bash
groups
```

The output must include `ethercat`.

## 9. Start and Verify IgH

Power the drive and connect the cable to `ECAT IN`, then:

```bash
sudo /etc/init.d/ethercat restart
ls -l /dev/EtherCAT0
/opt/etherlab/bin/ethercat master
/opt/etherlab/bin/ethercat slaves
```

Expected device permissions:

```text
crw-rw---- 1 root ethercat ... /dev/EtherCAT0
```

Expected slave:

```text
0  0:0  PREOP  +  AZD3A-KED rev0301
```

Useful read-only commands:

```bash
/opt/etherlab/bin/ethercat master
/opt/etherlab/bin/ethercat slaves
/opt/etherlab/bin/ethercat pdos -p 0
/opt/etherlab/bin/ethercat sdos -p 0
/opt/etherlab/bin/ethercat upload -p 0 --type uint16 0x603f 0
/opt/etherlab/bin/ethercat upload -p 0 --type uint16 0x6041 0
/opt/etherlab/bin/ethercat upload -p 0 --type int32 0x6064 0
```

Large listings can be saved instead of copied from terminal scrollback:

```bash
/opt/etherlab/bin/ethercat pdos -p 0 > azd3a_pdos.txt
/opt/etherlab/bin/ethercat sdos -p 0 > azd3a_sdos.txt
```

## 10. Recreate the ROS 2 Workspace

Clone the main project repository first. From its root, import the pinned
external dependency:

```bash
mkdir -p ~/kyutech/azd3a_ws/src
cd ~/kyutech/azd3a_ws
vcs import src < ~/kyutech/motor/Yuzu_code-Image-processing-motor-/azd3a_ws.repos
ln -s ~/kyutech/motor/Yuzu_code-Image-processing-motor-/motion_cmd src/motion_cmd
```

The selected driver branch expects `/usr/local/etherlab`, so provide a
compatibility link to the actual installation:

```bash
sudo ln -s /opt/etherlab /usr/local/etherlab
```

If the link already exists and points to `/opt/etherlab`, leave it unchanged.

Build:

```bash
cd ~/kyutech/azd3a_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Verify:

```bash
ros2 pkg prefix ethercat_driver
ros2 pkg prefix ethercat_generic_slave
ros2 pkg prefix ethercat_generic_cia402_drive
ros2 pkg prefix motor_controller
```

## 11. Commands Worth Memorizing

Environment:

```bash
uname -r
mokutil --sb-state
ip -brief link
groups
source /opt/ros/humble/setup.bash
source ~/kyutech/azd3a_ws/install/setup.bash
```

EtherCAT:

```bash
sudo /etc/init.d/ethercat restart
/opt/etherlab/bin/ethercat master
/opt/etherlab/bin/ethercat slaves
/opt/etherlab/bin/ethercat pdos -p 0
```

ROS:

```bash
ros2 pkg prefix ethercat_driver
ros2 pkg prefix ethercat_generic_cia402_drive
ros2 run motor_controller azd3a_ethercat_check --dry-run
```

Build:

```bash
cd ~/kyutech/azd3a_ws
colcon build --symlink-install
source install/setup.bash
```

## 12. Troubleshooting Map

| Symptom | Likely layer | First check |
| --- | --- | --- |
| `/dev/EtherCAT0` missing | IgH service/modules | `modinfo ec_master`; restart service |
| `Key was rejected by service` | Secure Boot | `mokutil --sb-state` |
| `Permission denied` | udev/group session | `groups`; `ls -l /dev/EtherCAT0` |
| Zero slaves | cable/power/NIC | link LEDs; `ethercat master`; `ip link` |
| `ETHERCAT_LIB-NOTFOUND` | install-prefix mismatch | `/usr/local/etherlab -> /opt/etherlab` |
| ROS package not found | workspace not sourced | `source install/setup.bash` |
| `ethercat_generic_plugins` not found | wrong package name | use `ethercat_generic_slave` and `ethercat_generic_cia402_drive` |
| Works until kernel upgrade | modules built for old kernel | rebuild IgH modules for new `uname -r` |

## 13. Windows Development Workflow

Windows Codex can review and edit the main repository normally. It should read:

1. `progress.md`
2. this guide
3. `motion_cmd/AZD3A_ETHERCAT_DRIVER_TEST.md`
4. `motion_cmd/config/azd3a_ked_cia402_slave.yaml`
5. `motion_cmd/AZD3A_HARDWARE.md`
6. `vendor/oriental_motor/README.md`

Nothing important should live only in `azd3a_ws`. That workspace is an Ubuntu
build environment containing an external dependency and generated artifacts.
When Ubuntu-specific code is added, keep its source in the main repository and
expose it to `azd3a_ws/src` with the existing symlink.

Windows cannot validate the IgH kernel modules, `/dev/EtherCAT0`, NIC ownership,
real-time behavior, or physical drive state. Those tests must remain on native
Ubuntu hardware. Windows review is suitable for Python, YAML, documentation,
unit tests, ROS configuration structure, and Git operations.

Do not commit `build`, `install`, or `log` from any colcon workspace.

The official ESI and EtherCAT manual are intentionally kept under
`vendor/oriental_motor/`. Use the ESI entry matching the live identity rather
than copying object IDs from memory. Connected-axis models and scaling cautions
are maintained in `motion_cmd/AZD3A_HARDWARE.md`.

## 14. Presentation-ready Milestones

1. Selected a dedicated NIC and configured IgH.
2. Built kernel modules matching the running Ubuntu kernel.
3. resolved Secure Boot module validation.
4. Added safe group-based access to `/dev/EtherCAT0`.
5. Discovered `AZD3A-KED rev0301` with zero frame loss.
6. Read CiA 402 PDO/SDO data and confirmed no drive error.
7. Built the Humble EtherCAT ROS 2 driver and CiA 402 plugin.
8. Next: minimal one-axis OP-state test, followed later by a tiny safe move.
