# Daily startup and troubleshooting

Use Ubuntu 22.04 with ROS 2 Humble and the configured IgH EtherCAT master.
First-time installation is described in `UBUNTU_ETHERCAT_SETUP_GUIDE.md` at
the bundle root. Keep the extracted source folder in its permanent location.

## Start the integrated GUI

1. Clear the mechanism and keep the physical emergency power cutoff accessible.
2. Connect the configured dedicated Ethernet NIC to the two-controller chain
   and confirm the required controller/motor power.
3. With no ROS motor-control session running, start and inspect EtherCAT:

```bash
sudo /etc/init.d/ethercat start
/opt/etherlab/bin/ethercat master
/opt/etherlab/bin/ethercat slaves
```

Confirm link up and two AZD3A controllers. Then, from the extracted bundle root:

```bash
bash scripts/start_yuzu_peeler.sh --workspace "$HOME/yuzu_ws"
```

Use the absolute workspace location selected during installation. Run only one
motor-control launch at a time. Wait for fresh feedback and all six drives in
Operation Enabled before commanding motion. Never restart EtherCAT while the
control session is running.

## Common problems

| Symptom | Check |
|---|---|
| Link down or zero slaves | Controller power, cable and the actual configured NIC; use `ip link` to identify it. |
| Missing `/dev/EtherCAT0` | Start the master and follow the setup guide for matching kernel modules. |
| Permission denied | Check the `ethercat` group and device permissions described in the setup guide. |
| Workspace not built / package not found | Complete `scripts/setup_workspace.sh` using the same workspace path. |
| GUI motion remains locked | Inspect the named motor's feedback and drive state; resolve the fault before motion. |
| Slaves repeatedly leave OP | Inspect individual slave states and drive diagnostics; an incomplete cyclic PDO exchange can cause watchdog faults. |
| Motor 4 accepts commands but does not move | Preserve its supplied feedback-based startup guard; a finite plugin startup latch can override guarded targets. |

Stop the ROS control session before low-level drive diagnostics. Do not change
PDO mappings, issue direct motion commands or bypass the guards during normal
operator troubleshooting. Preserve the reported drive fault for diagnosis.

## Stop

Use the GUI rotary stop for controlled stopping of Motors 2 and 6. It is not
an emergency stop. Closing the GUI sends rotary zero commands; position axes
hold their last guarded targets until the backend shuts down. End the launch
with Ctrl+C once the mechanism is safely stopped. Follow the machine's physical
shutdown procedure before servicing it.
