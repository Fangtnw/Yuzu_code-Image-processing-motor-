# Physical axis wiring for the integrated launch

This is the current harness assignment after the operator restored the CN4/CN5
EtherCAT cable order. A Motor 3 command turning Motor 2 exposed the temporary
controller-position swap in the integrated xacro. The six public
`/yuzu/motorN` names identify machine **roles**, not controller port numbers.
Axis 1–3 belong to EtherCAT slave position 0; Axis 4–6 belong to slave
position 1. Each AZD3A-KED has
three local axes with CiA-402 object offsets 0x0000, 0x0800, 0x1000.

| Physical axis | Slave position | Local axis | Logical motor | Machine role |
|---|---:|---:|---:|---|
| 1 | 0 | 1 | 1 | Vertical placement |
| 2 | 0 | 2 | 2 | Yuzu spin |
| 3 | 0 | 3 | 6 | Conveyor (formerly physical Axis 6) |
| 4 | 1 | 1 | 4 | Peeling-depth slide |
| 5 | 1 | 2 | 3 | Gripping/feed linear slide (formerly physical Axis 3) |
| 6 | 1 | 3 | 5 | Rotary peeler index (formerly physical Axis 5) |

The integrated launch is `yuzu_peeler.launch.py`. Its physical PDO mapping is
in `config/azd3a_motor1_motor2_slave.yaml` (slave 0) and
`config/azd3a_motor4_motor6_slave.yaml` (slave 1). The xacro
`urdf/azd3a_motor1_motor2.urdf.xacro` declares the matching command/state
interfaces and CiA-402 modes. `config/azd3a_motor1_motor2_controllers.yaml`
assigns each logical controller to a composite joint. The guards and GUI
select their feedback by joint and interface name. The separately named
commissioning launch files retain their historical wiring and must not be
used as proof of the current integrated port assignment.

## When the harness changes again

1. Record each physical port's **motor model, machine role, drive electronic
   gear settings, direction, and EtherCAT slave position/alias** in the table.
   Verify these on the actual drives. Do not move an old scaling factor to a
   different motor merely because it now occupies that port.
2. Change the two integrated slave YAML files: the local-axis PDO indices and
   object indices must follow the *port*; the command/state interface names,
   scaling, and status names must follow the *motor role*. Keep one command and
   one feedback source for each motor.
3. Change the integrated xacro's interface declarations and secondary/tertiary
   mode and object offsets, then the controller YAML's joint assignments.
   Update each guard's feedback joint and interface names, the integrated
   launch parameters, and the GUI's joint/interface lookup.
4. Run the offline suite, expand the xacro, and inspect every command/state
   interface using `ros2 control list_hardware_interfaces`. With power safely
   controlled, verify feedback identity and direction for each port before
   enabling a motion command. Recheck limits and startup behavior after a
   motor model or electronic gear change.

The old commissioning records in `AZD3A_HARDWARE.md` remain historical
measurements; they do not establish the new harness's live electrical or
mechanical behavior.
