# Yuzu horizontal diameter preview

The camera measurement is the horizontal diameter of one yuzu. Motors 3 and 4
use it as a reference for grasp and peel positions. Motor 5 follows the fixed
machine sequence; it does not follow individual camera contour rays.

The preview subscribes to `/yuzu/yellow_dot_offsets`
(`std_msgs/msg/Float32MultiArray`). The detector message contains 11 triples
`[angle_deg, dx_mm, dy_mm]` at 30°, 60°, ..., 330°. X points right, Y points
up, and angles increase clockwise from the top. The 90° and 270° points give
the right and left horizontal radii. The detector reports each point after a
radial inset, currently 3 mm, so:

```text
horizontal_diameter_mm = hypot(dx_90, dy_90) + hypot(dx_270, dy_270)
                         + 2 * source_inset_mm
```

The camera scale and inset must match the actual detector installation. A
diameter alone does not define either absolute slide position: the calibrated
blade gap at a known motor position, fruit centering, grasp clearance, and peel
depth are still needed. The preview publishes no motor commands.

## Camera-free check

Stop the real detector before starting the mock, since both publish on the
same topic. After building and sourcing `motor_controller`, use separate
terminals:

```bash
ros2 run motor_controller yuzu_mock_contour
ros2 run motor_controller yuzu_contour_preview
ros2 topic echo /yuzu/contour_preview --once --field data
```

The default mock has a 30 mm horizontal radius, 27 mm vertical radius, and
3 mm source inset. The preview should report `horizontal_diameter_mm: 60.0`,
`valid: true`, and `motion_enabled: false`. Changing only the mock's vertical
radius must leave the horizontal diameter at 60 mm. A mock with a 25 mm
horizontal radius should report 50 mm:

```bash
ros2 run motor_controller yuzu_mock_contour --ros-args -p horizontal_radius_mm:=25.0
```

Set the preview's `source_inset_mm` parameter to the detector's actual inset.
If no message arrives for one second, the preview reports `valid: false`.
The input has no camera capture timestamp, so freshness means only time since
ROS receipt. Use one visible fruit and one publisher.
