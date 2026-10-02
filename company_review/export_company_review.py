#!/usr/bin/env python3
"""Create a curated, source-only handoff tree for company review."""

from pathlib import Path
import argparse
from datetime import datetime
import shutil
import re
import hashlib


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
EXPORT_ROOT = Path(__file__).resolve().parent / "export" / "YuzuPeeler-MotorControl-current"

INCLUDED_PATHS = (
    "README.md",
    "LICENSE",
    "THIRD_PARTY_NOTICES.md",
    "patches",
    "azd3a_ws.repos",
    "UBUNTU_ETHERCAT_SETUP_GUIDE.md",
    "scripts/setup_workspace.sh",
    "scripts/prepare_driver.sh",
    "scripts/start_yuzu_peeler.sh",
    "motion_cmd",
    "tests",
    "vendor",
)


def validate_export(root: Path) -> None:
    """Reject personal paths and development artifacts in the release tree."""
    forbidden = {".git", ".claude", ".codex", ".agents", "build", "install", "log",
                 "__pycache__"}
    pattern = re.compile(r"/home/[\w.-]+|/Users/[\w.-]+|"
                         r"(?i:\bfang\b|kyutech|chatgpt|codex|claude)")
    failures = []
    for path in root.rglob("*"):
        if forbidden.intersection(path.relative_to(root).parts):
            failures.append(str(path.relative_to(root)))
        if path.is_file() and path.suffix.lower() in {
            ".md", ".py", ".sh", ".yaml", ".yml", ".xml", ".xacro", ".patch", ".repos"
        }:
            if pattern.search(path.read_text(errors="replace")):
                failures.append(str(path.relative_to(root)))
    if failures:
        raise ValueError("Export rejected; review: " + ", ".join(sorted(set(failures))))


def copy_source_tree(source: Path, destination: Path) -> None:
    if source.is_dir():
        shutil.copytree(
            source,
            destination,
            ignore=shutil.ignore_patterns(
                "__pycache__",
                "*.pyc",
                "*.pyo",
                "*.egg-info",
                ".DS_Store",
                ".git",
                ".claude",
                ".codex",
                ".agents",
                "build",
                "install",
                "log",
            ),
        )
    else:
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="destination directory (must not already exist; defaults to a fresh timestamped folder)",
    )
    arguments = parser.parse_args()
    output_root = arguments.output.resolve() if arguments.output else EXPORT_ROOT
    if arguments.output is None and output_root.exists():
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        output_root = output_root.with_name(f"{output_root.name}-{timestamp}")
    if output_root.exists():
        raise FileExistsError(
            f"Export destination already exists: {output_root}; choose a new --output path"
        )
    output_root.mkdir(parents=True)
    for relative_path in INCLUDED_PATHS:
        source = REPOSITORY_ROOT / relative_path
        if not source.exists():
            raise FileNotFoundError(f"Required handoff input is missing: {source}")
        copy_source_tree(source, output_root / relative_path)

    validate_export(output_root)
    checksums = []
    for path in sorted(output_root.rglob("*")):
        if path.is_file():
            checksums.append(hashlib.sha256(path.read_bytes()).hexdigest()
                             + "  " + path.relative_to(output_root).as_posix())
    (output_root / "SHA256SUMS").write_text("\n".join(checksums) + "\n")
    print(f"Company review source exported to: {output_root}")
    print("Review vendor-document terms and machine-specific setup before sharing.")


if __name__ == "__main__":
    main()
