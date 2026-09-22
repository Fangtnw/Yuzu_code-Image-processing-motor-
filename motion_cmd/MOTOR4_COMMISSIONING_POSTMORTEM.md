# Motor 4 EtherCAT and Combined-GUI Commissioning Post-mortem

Date: 2026-09-21

## Summary

Logical Motor 4 (`DR28T1A03-AZAKR`) was commissioned on AZD3A slave 1,
local Axis 1 (CN7), then integrated with Motors 1 and 2 in the two-controller
operator GUI. Two software defects appeared during integration: the standalone
slave-1 launch allowed upstream slave 0 to trip its SyncManager watchdog, and
the first combined launch left Motor 4's CSP startup latch permanently
overriding valid guarded commands. The standalone launch now supplies a safe
passive keepalive to slave 0, and the combined launch delegates Motor 4 startup
synchronization to its feedback-aware application guard. The operator has
physically confirmed that Motor 4 now moves from the combined GUI while Motors
1 and 2 remain controllable.

## Symptoms

The commissioning session produced these distinct symptoms:

1. Motor 4 initially reported alarm `0x0042`, status `0x0238`, and position
   zero. The motor/ABZO cable had not been connected to CN7 on controller #2.
2. The EtherCAT master later reported `Link: DOWN` and zero slaves after the
   laptop LAN connector was adjusted. Reseating the laptop connection restored
   both slaves.
3. The standalone Motor 4 launch repeatedly printed aggregate master states
   `0x08` and `0x0C`. High-rate sampling showed slave 0 cycling through OP,
   INIT, SAFEOP, and SAFEOP+ERROR while slave 1 stayed OP.
4. In the first combined GUI run, the Motor 4 guard logged an accepted target
   and `/motor4_raw_position_controller/commands` published `0.002 m`, but
   drive object `0x607A` remained equal to feedback at approximately 4,965
   counts. Motor 4 did not move although both slaves were OP and the domain
   working counter was 6/6.

## Root cause

The hardware faults had direct connection causes. Alarm `0x0042` was the AZD3A
ABZO sensor-at-power-on alarm caused by the missing Motor 4 CN7 connection.
The zero-slave event was a physical laptop Ethernet link loss, confirmed by the
master's `Link: DOWN` state.

The repeated AL-state transition was a launch topology defect. The original
Motor 4-only configuration mapped cyclic PDOs only for slave 1 while the IgH
master requested both physical slaves into OP. Slave 0 consequently received
no cyclic output for its active SyncManager. EtherCAT AL register `0x0134`
reported code `0x001B` (SyncManager watchdog). Slave 1 and its Motor 4 domain
remained healthy at WorkingCounter 3/3.

The combined-GUI no-motion defect was in the interaction between
`EcCiA402Drive::processData()` and concurrent controller startup. A finite
`position_startup_tolerance` keeps `channel.override_command` true until the
position command matches measured feedback within the configured tolerance.
For Motor 4, that latch never armed during the combined launch. The plugin
therefore replaced every valid guarded target with current feedback. Increasing
the tolerance from one encoder count to one actuator command increment did not
change the behavior and was rejected as the final fix.

## Why the defects produced the symptoms

An EtherCAT slave with an active output SyncManager must receive valid cyclic
frames. Slave 0 was physically upstream of slave 1 and could forward frames,
so Motor 4 continued to work, but slave 0's missing process data repeatedly
expired its watchdog. The master aggregates slave AL states, producing the
visible `0x08`/`0x0C` alternation.

In the combined GUI case, the complete ROS path worked: GUI publisher, Motor 4
guard, raw position controller, and claimed hardware interface were all live.
The CSP plugin's final write stage nevertheless selected measured position as
the PDO default while `position_command_synchronized_` was false. This explains
why ROS showed `0.002 m` on the raw command topic while `0x607A` and `0x6064`
both stayed at 4,965 counts.

## Fix

The standalone launch now declares two physical drives and maps slave 0 with
`GenericEcSlave` using `azd3a_slave0_passive_keepalive.yaml`. Its controlword,
target, and mode defaults remain zero, so the keepalive cannot enable Motors 1
or 2. Slave 1 remains the only CiA 402 motion device in that launch.

The combined launch maps both real slaves: the existing composite Motor 1/2
module on slave 0 and Motor 4 on slave 1. Motor 4's finite plugin startup latch
was removed from `azd3a_motor1_motor2.urdf.xacro`. Startup safety instead lives
in `azd3a_motor4_position_guard.py`, which continuously follows valid feedback
before the first operator command and rejects commands until feedback exists.
It also enforces absolute positions from 0 to 15 mm in 0.001 mm increments.

At the time of this incident, Motor 4 speed was distance-adaptive and remained below the actuator's catalog
limits:

- moves through 1 mm: 2 mm/s, 0.020 m/s²;
- moves through 5 mm: 5 mm/s, 0.050 m/s²;
- longer moves through the 15 mm software limit: 10 mm/s, 0.100 m/s².

## How it was found

Read-only SDO checks first separated motor alarms from EtherCAT transport.
Sampling `ethercat slaves` at 100 ms intervals identified slave 0 as the only
state-changing device. Reading AL status register `0x0134` captured `0x001B`,
confirming the SyncManager watchdog and disproving the physical-link hypothesis;
the master simultaneously reported zero lost frames.

For the GUI failure, topic endpoint inspection proved the GUI-to-guard and
guard-to-controller connections. The raw topic contained `0.002 m`, all three
controllers were active and claimed, both slaves were OP, and the domain was
6/6. Direct SDO reads still showed `0x607A == 0x6064 == 4,965`. Source tracing
then isolated `channel.override_command` in `EcCiA402Drive::processData()`.
The failed tolerance-increase retest proved the issue was a persistently
unarmed latch rather than insufficient numerical tolerance.

## Why it slipped through

The earlier tests exercised either one physical slave or Motor 4 alone. They
did not cover a two-slave chain where only the downstream slave had cyclic PDOs.
Static configuration tests verified addressing, scaling, topics, and limits,
but they could not exercise runtime CiA 402 startup ordering across multiple
controller spawners. The combined GUI was the first workload to cover both
gaps.

## Validation

Hardware and runtime evidence recorded during commissioning:

- Motor 4 electronic gearing: A=1, B=1.
- Scaling: 10,000 counts/mm.
- Alarm after connecting CN7: `0x0000`; stopped status: `0x0270`.
- Guarded +0.100 mm test: raw -37 to approximately 960 and return to -34.
- Guarded +0.500 mm test: exact raw 4,966; guarded recovery returned exactly
  to raw -34; alarm remained `0x0000`.
- Two-slave combined domain: WorkingCounter 6/6; both slaves OP.
- Combined GUI: Motors 1 and 2 remained controllable, and the operator
  physically confirmed Motor 4 movement after removal of the finite latch.
- Repository configuration tests: 19 passed.
- EtherCAT driver/workspace test result: 50 tests, zero errors and failures,
  six skipped.

The final combined Motor 4 position/count endpoint was not captured after the
operator's physical success confirmation. Motor 3 remains disconnected and is
not covered by this validation.

## Action items and follow-ups

- Capture Motor 4 `0x607A`, `0x6064`, and `/joint_states` at several GUI
  targets up to 15 mm as the assembled mechanism is cleared for travel.
- Motor 3 has since been reconnected on slave 0 Axis 3; complete read-only
  verification and third-state-machine driver integration before enabling it.
- Add a runtime/integration test for two physical EtherCAT slaves; static tests
  cannot reproduce SyncManager watchdogs or CSP startup ordering.
- Replace chain-position addressing with persistent EtherCAT aliases if the
  final installation supports them.
