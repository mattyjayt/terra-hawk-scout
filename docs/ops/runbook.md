---
id: RB-SCOUT
title: Scout runbook
status: Draft
owner: Argos (draft) · Jaime (approver)
updated: 2026-11-09
---

# RB-SCOUT — runbook
1. **Pi 5 setup:** Ubuntu 24.04 + ROS 2 Jazzy. Build steps: root [README](../../README.md#quick-start).
2. **R4 base link:** plug the UNO R4 WiFi into a Pi USB port; copy [`tools/udev/99-scout-base.rules`](../../tools/udev/99-scout-base.rules) to `/etc/udev/rules.d/` → `/dev/scout_base` (USB ID `2341:1002`; edit the serial number for your board).
3. **Flash firmware:** `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi firmware/r4_base`, then `arduino-cli upload -p /dev/scout_base --fqbn arduino:renesas_uno:unor4wifi firmware/r4_base` (or the Arduino IDE on the Mac).
4. **Bring-up:** start the camera (`ros2 run camera_ros camera_node …`, see the README), then `ros2 launch scout_bringup base.launch.py` (bridge, robot model, Foxglove, perception).
5. **Recover:** USB serial drop / R4 reset, DDS discovery issues, undervoltage (`vcgencmd get_throttled`) — TBD
