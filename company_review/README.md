# Company source-review package

This folder contains the export utility and instructions for creating a clean,
self-contained source-review package. The export is generated from the working
tree so it does not require maintaining a second copy of the application.

From the repository root, run:

```bash
python3 company_review/export_company_review.py
```

The generated package is written to
`company_review/export/` in a fresh `YuzuPeeler-MotorControl-current-*` folder.
It includes the ROS 2 package,
tests, setup scripts, deployment guide, and vendor references. Internal
progress logs, superseded planning notes, and editor-specific files are not
included. Review the generated tree and applicable vendor-document terms before
sharing it outside the organization.

Previous exports are kept. Pass `--output /path/to/new-folder` to choose a
specific destination; it must not already exist.

The package is a source-review snapshot, not a validated customer deployment.
Hardware-specific EtherCAT configuration, NIC setup, kernel modules, motor
state, and mechanical safety must be verified on the target system.

Before external distribution, confirm the approved maintainer name/contact and
the project owner's license choice. The ROS package currently declares MIT,
and the top-level LICENSE now supplies that text. See THIRD_PARTY_NOTICES.md
for the placeholder contact and third-party redistribution checks.

Send only the newly validated export, never the entire export directory.
Older snapshots are retained for local reference and may contain development
artifacts. The exporter rejects personal paths and development-tool markers
and writes SHA256SUMS. Setup includes the pinned multi-axis driver patch.
