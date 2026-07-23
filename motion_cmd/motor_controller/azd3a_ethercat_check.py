"""Python check controller for one AZD3A-KED EtherCAT basic test."""

import argparse
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Check:
    label: str
    command: list[str]
    required_text: tuple[str, ...] = ()


CHECKS = (
    Check("Find IgH EtherCAT slave", ["ethercat", "slaves"], ("AZD",)),
    Check("Read EtherCAT PDO map", ["ethercat", "pdos"], ("0x6040", "0x6041")),
    Check("Read EtherCAT SDO dictionary", ["ethercat", "sdos"]),
    Check("Find ROS2 ethercat_driver package", ["ros2", "pkg", "prefix", "ethercat_driver"]),
    Check(
        "Find ROS2 ethercat_generic_plugins package",
        ["ros2", "pkg", "prefix", "ethercat_generic_plugins"],
    ),
)

ETHERCAT_CONFIG = Path("/etc/sysconfig/ethercat")


def _format_command(command: list[str]) -> str:
    return " ".join(command)


def _run_check(check: Check, dry_run: bool) -> bool:
    print(f"\n== {check.label} ==")
    print(f"$ {_format_command(check.command)}")

    if dry_run:
        return True

    executable = check.command[0]
    if shutil.which(executable) is None:
        print(f"FAIL: '{executable}' command was not found in PATH.")
        return False

    result = subprocess.run(
        check.command,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    output = result.stdout.strip()
    if output:
        print(output)

    if result.returncode != 0:
        print(f"FAIL: command exited with code {result.returncode}.")
        return False

    missing = [text for text in check.required_text if text not in output]
    if missing:
        print(f"FAIL: expected output to contain: {', '.join(missing)}")
        return False

    print("PASS")
    return True


def _read_ethercat_config_value(text: str, key: str) -> str | None:
    match = re.search(rf"^\s*{re.escape(key)}\s*=\s*\"([^\"]*)\"", text, re.MULTILINE)
    return match.group(1) if match else None


def _print_igh_config_hints() -> None:
    if not ETHERCAT_CONFIG.exists():
        print(f"- IgH config file is missing: {ETHERCAT_CONFIG}")
        return

    try:
        text = ETHERCAT_CONFIG.read_text(encoding="utf-8")
    except OSError as exc:
        print(f"- Could not read {ETHERCAT_CONFIG}: {exc}")
        return

    master0_device = _read_ethercat_config_value(text, "MASTER0_DEVICE")
    device_modules = _read_ethercat_config_value(text, "DEVICE_MODULES")

    if master0_device == "":
        print(f"- {ETHERCAT_CONFIG}: set MASTER0_DEVICE to the EtherCAT NIC MAC or interface name")
    if device_modules == "":
        print(f"- {ETHERCAT_CONFIG}: set DEVICE_MODULES, usually \"generic\" for first bring-up")


def _print_next_steps(success: bool, dry_run: bool) -> None:
    print("\n== Result ==")
    if dry_run:
        print("Dry run complete. No EtherCAT or ROS2 commands were executed.")
        print("Run without --dry-run on the Ubuntu ROS2 computer with one AZD3A-KED connected.")
        return

    if success:
        print("Basic EtherCAT check passed.")
        print("Next: compare the PDO objects with config/azd3a_ked_cia402_slave.yaml.")
        print("Do not send motion yet; this only proves discovery and package availability.")
        return

    print("Basic EtherCAT check failed.")
    print("Check these first:")
    print("- AZD3A-KED control power and main power")
    print("- Cable from Ubuntu spare NIC to ECAT IN")
    print("- IgH master service: sudo /etc/init.d/ethercat start")
    print("- Correct NIC MAC in /etc/sysconfig/ethercat")
    _print_igh_config_hints()
    print("- ROS2 workspace is sourced: source install/setup.bash")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the minimum one-axis AZD3A-KED EtherCAT checks."
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print checks without running commands.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)

    print("AZD3A-KED EtherCAT basic check")
    print("This checks discovery and package availability only; it does not move the motor.")

    results = [_run_check(check, args.dry_run) for check in CHECKS]
    success = all(results)
    _print_next_steps(success, args.dry_run)
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
