# SCOUT-ROS 101 — ROS 2 on a real robot (lab course)

> Personal learning material for Jaime. **Not formal documentation**: the formal docs are in `docs/specs/` and `docs/design/`. This folder will be removed later.

## Course goal
Take the Scout from "an Arduino that drives when you type `M 100 100`" to a ROS 2 robot: a base bridge written twice (Python and C++), tested at three levels, visualised live in Foxglove, described by a URDF, and finally a camera that keeps a person in view with the pan servo.

## How the course works
- **The formal contract is the assignment.** Every lab implements something that is already specified:
  - [IF-ROS-TOPICS](../specs/interfaces/ros-topics.md): names, types, units, QoS.
  - [DD-SCOUT-BRIDGE](../design/DD-SCOUT-BRIDGE.md): how the bridge works, plus its test design.
  - [serial-r4-v1](../specs/interfaces/serial-r4-v1.md): what the R4 speaks.

  When the lab text and the contract disagree, the contract wins. Tell Argos so the lab gets fixed.
- **You write the code.** Argos explains, reviews, and builds the plumbing marked 🔧. Tasks marked 🧠 are learning-critical: no copy-paste from the internet or AI, and you must be able to defend every line in an interview.
- **Tests before code** in Labs 3, 7 and 8 (RED → GREEN → REFACTOR).
- **Every task ends with a Check.** It's done when the check passes, not when the code compiles.
- **Lab report** per lab: `docs/learning/reports/lab-N.md` (template below). Short: results table, answers to the questions, what broke.

## Labs
| Lab | Topic | Est. time | Today (3 Nov)? |
|---|---|---|---|
| [1](lab-1-workspace.md) | Workspace, packages, VS Code, udev | 45 min | ✅ |
| [2](lab-2-bench.md) | Bench characterisation: IMU fault, axes, servo, battery, drive calibration | 1.5 h | ✅ |
| [3](lab-3-python-core.md) | Python bridge core with unit tests (TDD) | 1.5 h | ✅ |
| [4](lab-4-python-node.md) | Python bridge node + software-in-the-loop tests | 2–3 h | ✅ (stretch) |
| [5](lab-5-foxglove-hitl.md) | Foxglove, teleop, hardware-in-the-loop, rosbag, recording | 1.5 h | if time |
| [6](lab-6-urdf.md) | Robot model: URDF, TF, bringup launch | 1.5 h | next |
| [7](lab-7-cpp-bridge.md) | The same bridge in C++ (rclcpp, gtest), shared contract tests | 3–4 h | next |
| [8](lab-8-vision.md) | Vision: camera, person detection (YOLO11n, NCNN), pan tracking | 3–4 h | after the camera fix |

## Ground rules on the robot
1. **Wheels off the ground** (box under the chassis) for every first run of new drive code.
2. Battery switch within reach; USB unplug = emergency stop (the R4 stops in 300 ms).
3. One terminal per process; name your tmux panes or VS Code terminals.
4. Commit small and often on a branch (`feat/lab-N-...`), and open a PR per lab for review.

## Environment facts
| | |
|---|---|
| Pi | `ssh ghorman` (user `kleya`), Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, numpy 1.26, OpenCV 4.11 |
| Repo on the Pi | `~/Projects/terra-hawk-scout`, ROS workspace `ros2_ws/` inside it |
| R4 | USB ID `2341:1002`, serial `F412FA9FE1BC`, today on `/dev/ttyACM0` |
| Camera | Camera Module 3 (imx708), detected by `rpicam-hello`; ROS driver blocked (Lab 8) |
| Installed | `foxglove_bridge`, `teleop_twist_keyboard`, `camera_ros` (apt, broken for Pi 5), `vision_msgs`, `cv_bridge`, `robot_state_publisher`, `image_transport_plugins`, `launch_testing`, `ament_cmake_gtest`, `pyserial` |
| Missing (install when the lab says so) | `ros-jazzy-xacro`, `libserial-dev`, `ros-jazzy-robot-localization` |
| Fake R4 | `tools/fake_r4.py`: pty at `/tmp/fake_r4`, `--log` JSON, `--freeze-imu` |

## Reading list (used across the labs)
- ROS 2 Jazzy tutorials: https://docs.ros.org/en/jazzy/Tutorials.html (Beginner: Client Libraries; Intermediate: Launch, Testing)
- About QoS: https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html
- REP-103 (units, axes) https://www.ros.org/reps/rep-0103.html · REP-105 (frames) https://www.ros.org/reps/rep-0105.html · REP-145 (IMU conventions) https://www.ros.org/reps/rep-0145.html
- Lynch & Park, *Modern Robotics*, ch. 13.3 (wheeled mobile robots, kinematics). Free PDF at http://hades.mech.northwestern.edu/index.php/Modern_Robotics
- Articulated Robotics (Josh Newans): "Making a Mobile Robot" YouTube series and https://github.com/joshnewans/diffdrive_arduino, the same Pi ↔ Arduino-over-serial pattern taken to `ros2_control` (our DD-SCOUT-HWIF step).
- Gary Bernhardt, "Boundaries" (talk, 2012): the *functional core, imperative shell* idea behind DD-SCOUT-BRIDGE §3.
- Foxglove + ROS 2: https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2
- Ultralytics on Raspberry Pi: https://docs.ultralytics.com/guides/raspberry-pi/ · NCNN export: https://docs.ultralytics.com/integrations/ncnn/ · tracking: https://docs.ultralytics.com/modes/track/
- Zhang et al., *ByteTrack* (ECCV 2022), arXiv:2110.06864

## Lab report template
```markdown
# Lab N — <title> · <date> · commit <sha>
## Results
| Task | Check | Result (pass/fail, numbers) |
## Answers
1. …
## What broke and how I fixed it
## Open questions for Argos
```
