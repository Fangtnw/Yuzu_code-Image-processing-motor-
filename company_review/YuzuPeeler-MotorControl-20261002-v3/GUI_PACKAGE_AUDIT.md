# GUI distribution audit — 2026-10-02

Reviewed package: 
C:\Users\thana\01_Fang\Advance Practice\Yuzu_code-Image-processing-motor-\company_review\YuzuPeeler-MotorControl-20261002-v3\YuzuPeeler-MotorControl-20261002-v3

104 files, 17,919,013 bytes; motion_cmd contains 82 files. Review was static: launch references, local imports, packaging, scripts and text documentation. No GUI or motor commands were executed. PDF/image contents and embedded metadata were not audited.

## Findings

The package includes substantial development and commissioning material beyond integrated GUI operation. No obvious personal home paths, personal contact addresses, credentials, logs, Git history, virtual environments or bytecode caches were found in the inspected inventory/text. This is not a comprehensive secret or document-metadata scan.

Personal/internal wording: motion_cmd/AZD3A_ETHERCAT_DRIVER_TEST.md:3 says "your Person 2 role". AZD3A_HARDWARE.md:160 and YUZU_OPERATOR_GUI.md:162 refer to a supervisor request. These should be rewritten or moved to engineering documentation.

YUZU_OPERATOR_GUI.md:145 onward mixes historical commissioning evidence with old startup advice and differing Motor 6 defaults (10 and 50 mm/s). Current preceding documentation states a 75.4 mm/s default. Preserve validation limitations, but separate historical evidence from current instructions.

AZD3A_DAILY_STARTUP_TROUBLESHOOTING.md assumes NIC eno2 and lists standalone launches. Adapt it to the actual recipient NIC and integrated launch if retained.

setup.py and package.xml use maintainer@example.invalid; an approved company contact is needed for a supported release. The integrated URDF/config names still say motor1_motor2 although they serve all six motors; cosmetic renaming is optional and must update references.

## Integrated application keep list

These 19 files form the identified application source/build dependency set for yuzu_peeler.launch.py. External ROS/EtherCAT dependencies are additional. Keep concise operator/setup documentation and licenses alongside them.

- motion_cmd/setup.py
- motion_cmd/setup.cfg
- motion_cmd/package.xml
- motion_cmd/resource/motor_controller
- motion_cmd/motor_controller/__init__.py
- motion_cmd/motor_controller/yuzu_peeler_gui.py
- motion_cmd/motor_controller/azd3a_motor1_position_guard.py
- motion_cmd/motor_controller/azd3a_axis1_command_guard.py
- motion_cmd/motor_controller/azd3a_motor2_velocity_guard.py
- motion_cmd/motor_controller/azd3a_axis2_velocity_guard.py
- motion_cmd/motor_controller/azd3a_motor3_position_guard.py
- motion_cmd/motor_controller/azd3a_motor4_position_guard.py
- motion_cmd/motor_controller/azd3a_motor5_position_guard.py
- motion_cmd/motor_controller/azd3a_motor6_conveyor_guard.py
- motion_cmd/launch/yuzu_peeler.launch.py
- motion_cmd/urdf/azd3a_motor1_motor2.urdf.xacro
- motion_cmd/config/azd3a_motor1_motor2_controllers.yaml
- motion_cmd/config/azd3a_motor1_motor2_slave.yaml
- motion_cmd/config/azd3a_motor4_motor6_slave.yaml

Motor 1 imports azd3a_axis1_command_guard.py; Motor 2 imports azd3a_axis2_velocity_guard.py. Both older-named implementations are required. Keep all six guards.

## Files outside the integrated executable/configuration path

The following 54 source/configuration files are candidates to exclude from a GUI-only distribution. They support standalone tests, demonstrations or commissioning. Retain them in the engineering repository. Removing Python modules also requires removing their console_scripts entries from setup.py.

- motion_cmd/config/azd3a_axis1_feedback_controllers.yaml
- motion_cmd/config/azd3a_axis1_tiny_move_controllers.yaml
- motion_cmd/config/azd3a_axis2_feedback_controllers.yaml
- motion_cmd/config/azd3a_axis2_feedback_slave.yaml
- motion_cmd/config/azd3a_axis2_tiny_spin_controllers.yaml
- motion_cmd/config/azd3a_axis2_tiny_spin_slave.yaml
- motion_cmd/config/azd3a_axis3_feedback_controllers.yaml
- motion_cmd/config/azd3a_axis3_feedback_slave.yaml
- motion_cmd/config/azd3a_axis3_index_controllers.yaml
- motion_cmd/config/azd3a_axis3_index_slave.yaml
- motion_cmd/config/azd3a_axis3_tiny_spin_controllers.yaml
- motion_cmd/config/azd3a_axis3_tiny_spin_slave.yaml
- motion_cmd/config/azd3a_ked_cia402_slave.yaml
- motion_cmd/config/azd3a_motor3_commissioning_controllers.yaml
- motion_cmd/config/azd3a_motor3_commissioning_slave.yaml
- motion_cmd/config/azd3a_motor4_commissioning_controllers.yaml
- motion_cmd/config/azd3a_motor4_commissioning_slave.yaml
- motion_cmd/config/azd3a_motor5_commissioning_controllers.yaml
- motion_cmd/config/azd3a_motor5_commissioning_slave.yaml
- motion_cmd/config/azd3a_motor6_conveyor_controllers.yaml
- motion_cmd/config/azd3a_motor6_conveyor_slave.yaml
- motion_cmd/config/azd3a_slave0_passive_keepalive.yaml
- motion_cmd/config/azd3a_slave1_passive_keepalive.yaml
- motion_cmd/launch/azd3a_axis1_feedback.launch.py
- motion_cmd/launch/azd3a_axis1_tiny_move.launch.py
- motion_cmd/launch/azd3a_axis2_feedback.launch.py
- motion_cmd/launch/azd3a_axis2_tiny_spin.launch.py
- motion_cmd/launch/azd3a_axis3_feedback.launch.py
- motion_cmd/launch/azd3a_axis3_index.launch.py
- motion_cmd/launch/azd3a_axis3_tiny_spin.launch.py
- motion_cmd/launch/azd3a_motor3_commissioning.launch.py
- motion_cmd/launch/azd3a_motor4_commissioning.launch.py
- motion_cmd/launch/azd3a_motor5_commissioning.launch.py
- motion_cmd/launch/azd3a_motor6_conveyor.launch.py
- motion_cmd/motor_controller/azd3a_axis1_video_demo.py
- motion_cmd/motor_controller/azd3a_axis3_index_guard.py
- motion_cmd/motor_controller/azd3a_axis3_velocity_guard.py
- motion_cmd/motor_controller/azd3a_basic_commands.py
- motion_cmd/motor_controller/azd3a_basic_test_node.py
- motion_cmd/motor_controller/azd3a_ethercat_check.py
- motion_cmd/motor_controller/azd3a_motor3_commissioning_guard.py
- motion_cmd/motor_controller/azd3a_motor4_commissioning_guard.py
- motion_cmd/motor_controller/azd3a_motor5_commissioning_guard.py
- motion_cmd/urdf/azd3a_axis1_feedback.urdf.xacro
- motion_cmd/urdf/azd3a_axis1_tiny_move.urdf.xacro
- motion_cmd/urdf/azd3a_axis2_feedback.urdf.xacro
- motion_cmd/urdf/azd3a_axis2_tiny_spin.urdf.xacro
- motion_cmd/urdf/azd3a_axis3_feedback.urdf.xacro
- motion_cmd/urdf/azd3a_axis3_index.urdf.xacro
- motion_cmd/urdf/azd3a_axis3_tiny_spin.urdf.xacro
- motion_cmd/urdf/azd3a_motor3_commissioning.urdf.xacro
- motion_cmd/urdf/azd3a_motor4_commissioning.urdf.xacro
- motion_cmd/urdf/azd3a_motor5_commissioning.urdf.xacro
- motion_cmd/urdf/azd3a_motor6_conveyor.urdf.xacro

## Documentation and other material

- Move MOTOR4_COMMISSIONING_POSTMORTEM.md, AZD3A_BASIC_TEST.md and AZD3A_ETHERCAT_DRIVER_TEST.md to an engineering/reference archive outside the operator ZIP.
- Keep a revised YUZU_OPERATOR_GUI.md. Condense AZD3A_HARDWARE.md and AZD3A_DAILY_STARTUP_TROUBLESHOOTING.md into current hardware/setup/troubleshooting guidance.
- YuzuSequence.pdf and sequence.png are not loaded by the GUI; optional operator references, subject to content review.
- motion_cmd/.gitignore and the empty motion_cmd/tools directory are unnecessary for operation.
- tests/ is useful for engineering review but unnecessary for GUI startup.
- vendor/ PDFs/XML are not read by the integrated application. Keep source citations and useful hardware references separately; preserve the applicable notices.
- Keep scripts/, patches/ and azd3a_ws.repos for installation on a new PC. prepare_driver.sh applies the multi-axis driver patch; deleting it or patches/ breaks supplied setup.
- Keep README.md, the Ubuntu setup guide, LICENSE and applicable THIRD_PARTY_NOTICES.md for delivery.

## Required follow-through before a reduced ZIP

Update setup.py entry points, README layout/test/compatibility claims and documentation links to match exclusions. Regenerate SHA256SUMS after all changes; adjust vendor hashes if that subtree changes. Confirm a clean Ubuntu/ROS build and integrated launch on the target environment before treating a reduced package as operationally verified. Current folder is source requiring first-time installation, not a portable executable.

No package files were changed or removed by this audit.
