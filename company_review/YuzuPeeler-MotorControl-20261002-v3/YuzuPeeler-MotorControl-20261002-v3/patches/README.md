# EtherCAT driver extension

Base: ICube-Robotics/ethercat_driver_ros2 commit
`97c6dc3afbb89928901043ad06d1d1e5f7be104f` (see `azd3a_ws.repos`).

`ethercat-driver-multiaxis.patch` includes the header, implementation and
upstream test extensions used by the working installation. It adds independent
secondary and tertiary CiA-402 state machines, object-index offsets and position
startup synchronization. Each AZD3A-KED contains three physical motor axes.
The unmodified upstream revision is not an equivalent runtime.

`scripts/setup_workspace.sh` imports the pinned source and applies this patch
before building the application and its driver dependencies. Reapplying is safe;
a conflicting source revision or patch fails without overwriting local edits.
The upstream project is Apache-2.0 licensed; retain its LICENSE and notices.
Internet access is required to import the source and install dependencies.
