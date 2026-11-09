# Scout documentation index
Process: PRD → spec → interface specs → design docs → tasks → code → verification → evidence. No code before its design doc is **Approved**.
IDs: `G-SCOUT-n` goals · `REQ-SCOUT-nnn` requirements · `IF-*` interfaces · `DD-SCOUT-*` design · `ADR-nnnn` · `TC-SCOUT-nnn` tests.

| Doc | ID | Status |
|---|---|---|
| [Product requirements](product/prd.md) | PRD-SCOUT | Draft |
| [Software requirements spec](specs/srs.md) | SRS-SCOUT | Draft |
| [ROS 2 interface contract: topics, services, frames, QoS](specs/interfaces/ros-topics.md) | IF-ROS-TOPICS v1.0 | Approved (P1–P2) |
| [Pi ↔ R4 serial protocol v1](specs/interfaces/serial-r4-v1.md) | IF-SERIAL-R4-V1 | Implemented |
| [Hardware: layout, power, wiring, pinout](design/DD-SCOUT-HW.md) | DD-SCOUT-HW | Draft |
| [R4 base firmware](design/DD-SCOUT-FW.md) | DD-SCOUT-FW | Planned: v2 design (v1 firmware in use; issue #16) |
| [Base bridge node (Python + C++)](design/DD-SCOUT-BRIDGE.md) | DD-SCOUT-BRIDGE | Python implemented and tested; C++ in progress |
| [ros2_control hardware interface](design/DD-SCOUT-HWIF.md) | DD-SCOUT-HWIF | Deferred |
| [Perception node (Python)](design/DD-SCOUT-PERC.md) | DD-SCOUT-PERC | In progress (code in `scout_perception`) |
| [Localisation (LiDAR odometry + EKF)](design/DD-SCOUT-LOC.md) | DD-SCOUT-LOC | Planned |
| [SLAM and Nav2](design/DD-SCOUT-NAV.md) | DD-SCOUT-NAV | Planned |
| [Mission](design/DD-SCOUT-MISSION.md) | DD-SCOUT-MISSION | Planned |
| [Bill of materials](hardware/bom.md) | — | Draft |
| [Test plan](testing/test-plan.md) | TP-SCOUT | Draft |
| [Runbook](ops/runbook.md) | RB-SCOUT | Draft |
| [ADRs](adr/README.md) | ADR-0001…0006 | see index (0002 superseded, 0003 deferred) |

## Visual artifacts
Interactive HTML pages: download and open them in a browser.

| Artifact | What it shows |
|---|---|
| [P1/P2 ROS 2 graph](diagrams/scout-p1-graph/scout-p1-graph.html) | the current node graph: base bridge, robot model, camera, perception, Foxglove |
| [cmd_vel safety sequence](diagrams/scout-cmd-safety/scout-cmd-safety.html) | command timeout and R4 watchdog |
| [Twist → PWM figure](figures/twist-to-pwm/twist-to-pwm.html) | how the bridge turns `cmd_vel` into wheel PWM |
| [Sensor-fusion figure](figures/sensor-fusion/sensor-fusion.html) | concept: why fusing a gyro with odometry fixes heading drift (drawn with wheel encoders; on the Scout, LiDAR odometry takes that role) |
| [Layout fit check (CAD)](../cad/scout-layout/README.md) | v1 fit check on the Elegoo plates |
| [Early target graph](diagrams/scout-ros2-graph/scout-ros2-graph.html) | **superseded**: the pre-ADR-0006 design with an ESP32 and micro-ROS. Kept as history only |
