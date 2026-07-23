# EtherCAT Bring-up Progress

Date: 2026-07-23

## Goal

Bring up one AZD3A-KED EtherCAT axis on Ubuntu/ROS2 and verify basic discovery
before attempting motion.

## What Was Checked

Ran:

```bash
ros2 run motor_controller azd3a_ethercat_check
```

Initial result:

- `ethercat slaves`, `ethercat pdos`, and `ethercat sdos` failed because
  `/dev/EtherCAT0` did not exist.
- `ros2 pkg prefix ethercat_driver` failed with `Package not found`.
- `ros2 pkg prefix ethercat_generic_plugins` failed with `Package not found`.

## IgH Configuration

The EtherCAT command-line tool was installed at:

```bash
/opt/etherlab/bin/ethercat
```

The IgH config file existed at:

```bash
/etc/sysconfig/ethercat
```

It initially had empty values:

```bash
MASTER0_DEVICE=""
DEVICE_MODULES=""
```

The available network interfaces showed:

- `eno2`: wired Ethernet, MAC `a0:36:bc:31:3f:18`
- `wlo1`: Wi-Fi

Conclusion: use `eno2` for EtherCAT, not `wlo1`.

Recommended config:

```bash
MASTER0_DEVICE="eno2"
DEVICE_MODULES="generic"
UPDOWN_INTERFACES="eno2"
```

## Kernel Module Issue

After configuring IgH and restarting:

```bash
sudo /etc/init.d/ethercat restart
```

IgH failed with:

```text
modprobe: FATAL: Module ec_master not found in directory /lib/modules/6.8.0-124-generic
```

The running kernel was:

```bash
6.8.0-124-generic
```

Headers for this kernel were installed, but no `ec_master.ko` existed under
`/lib/modules`.

The IgH source checkout was found at:

```bash
~/kyutech/ethercat
```

It was already configured for:

```bash
--prefix=/opt/etherlab
--sysconfdir=/etc
--with-linux-dir=/usr/src/linux-headers-6.8.0-124-generic
--enable-generic
```

Built the missing modules successfully:

```bash
cd ~/kyutech/ethercat
make modules
```

Then installed them manually from the user shell:

```bash
sudo make modules_install
sudo depmod
```

`modinfo` confirmed:

- `ec_master` installed at
  `/lib/modules/6.8.0-124-generic/ethercat/master/ec_master.ko`
- `ec_generic` installed at
  `/lib/modules/6.8.0-124-generic/ethercat/devices/ec_generic.ko`

## Current Blocker

Restarting IgH now fails with:

```text
modprobe: ERROR: could not insert 'ec_master': Key was rejected by service
```

Secure Boot is enabled:

```bash
mokutil --sb-state
# SecureBoot enabled
```

Conclusion: the modules are installed, but the kernel refuses to load unsigned
third-party modules because Secure Boot validation is active.

## Next Step

Fast lab-bring-up option:

```bash
sudo mokutil --disable-validation
```

Reboot, use the blue MOK screen to disable validation, then run:

```bash
sudo /etc/init.d/ethercat restart
ls -l /dev/EtherCAT0
ethercat slaves
```

Alternative: keep Secure Boot enabled and sign `ec_master.ko` plus
`ec_generic.ko` with an enrolled Machine Owner Key.

## Repository Change

Updated `motion_cmd/motor_controller/azd3a_ethercat_check.py` so failed checks
also report concrete IgH config hints when `/etc/sysconfig/ethercat` leaves
`MASTER0_DEVICE` or `DEVICE_MODULES` blank.

Validation:

```bash
python3 -m unittest tests/test_azd3a_ethercat_config_files.py
```

Result: passed.
