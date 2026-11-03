# Scout documentation index
Process: PRD → spec → interface specs → design docs → tasks → code → verification → evidence. No code before its design doc is **Approved**.
IDs: `G-SCOUT-n` goals · `REQ-SCOUT-nnn` requirements · `IF-*` interfaces · `DD-SCOUT-*` design · `ADR-nnnn` · `TC-SCOUT-nnn` tests.

| Doc | ID | Status |
|---|---|---|
| [Product requirements](product/prd.md) | PRD-SCOUT | Draft |
| [Software requirements spec](specs/srs.md) | SRS-SCOUT | Draft |
| [ROS 2 topics, frames, QoS](specs/interfaces/ros-topics.md) | IF-ROS-TOPICS | Draft |
| [Pi ↔ R4 serial protocol v1](specs/interfaces/serial-r4-v1.md) | IF-SERIAL-R4-V1 | Implemented |
| [Hardware: layout, power, wiring, pinout](design/DD-SCOUT-HW.md) | DD-SCOUT-HW | Draft |
| [R4 base firmware](design/DD-SCOUT-FW.md) | DD-SCOUT-FW | Not started (Jaime writes it, issue #16) |
| [ros2_control hardware interface](design/DD-SCOUT-HWIF.md) | DD-SCOUT-HWIF | Not started (learn) |
| [Perception node](design/DD-SCOUT-PERC.md) | DD-SCOUT-PERC | Not started (learn) |
| [Localisation (EKF)](design/DD-SCOUT-LOC.md) | DD-SCOUT-LOC | Not started (learn) |
| [SLAM and Nav2](design/DD-SCOUT-NAV.md) | DD-SCOUT-NAV | Not started (learn) |
| [Mission](design/DD-SCOUT-MISSION.md) | DD-SCOUT-MISSION | Not started |
| [Bill of materials](hardware/bom.md) | — | Draft |
| [Test plan](testing/test-plan.md) | TP-SCOUT | Draft |
| [Runbook](ops/runbook.md) | RB-SCOUT | Stub |
| [ADRs](adr/README.md) | ADR-0001… | Accepted |

**Visual artifacts:** [ROS 2 graph (archify)](diagrams/scout-ros2-graph/scout-ros2-graph.html) · [Sensor-fusion figure (anatomy)](figures/sensor-fusion/sensor-fusion.html) · [Layout fit check (CAD)](../cad/scout-layout/README.md)
