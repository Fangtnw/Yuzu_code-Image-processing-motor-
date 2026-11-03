# ROS 2 discovery between motor and vision laptops

The motor laptop has EtherCAT-side `eno2` (`192.168.1.20`) and Wi-Fi `wlo1`
(`192.168.0.157`) at the time of diagnosis. Both computers use ROS domain 21.
The vision laptop was reachable by ping, and a `ros2 multicast send` from its
Wi-Fi IP `192.168.0.235` reached `ros2 multicast receive` on the motor laptop.

The camera node became visible on the motor laptop after the forced
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` setting was removed. For this setup,
use the ROS default middleware in the terminal that launches motor control:

```bash
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
export ROS_DOMAIN_ID=21
export ROS_LOCALHOST_ONLY=0
bash scripts/start_yuzu_peeler.sh --workspace "$HOME/yuzu_ws"
```

The launcher's optional `--ros-interface wlo1` setting forces Cyclone DDS and
should not be used with the currently running vision setup. If both computers
later use Cyclone DDS, explicitly selecting their respective Wi-Fi interfaces
may be useful. Environment changes affect newly started processes only.

Check the active graph without the ROS CLI daemon:

```bash
ros2 node list --no-daemon
ros2 topic list -t --no-daemon
```

On the vision PC, the detector already publishes yuzu topics including
diameter and yellow-dot data. Confirm their exact names and message types with
`ros2 topic list -t --no-daemon` before wiring the motor control subscriber.
Motor 1 now uses the detector's `/yuzu/y_offset_mm` (`std_msgs/msg/Float32`)
for its camera-based reference calculation. See `MOTOR1_VISION_HANDOFF.md`.
