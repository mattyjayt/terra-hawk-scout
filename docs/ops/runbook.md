---
id: RB-SCOUT
title: Scout runbook
status: Stub
owner: Argos (draft) · Jaime (approver)
updated: 2026-11-03
---

# RB-SCOUT — runbook
1. **Pi 5 setup** (Ubuntu 24.04, ROS 2 Jazzy: installed on `ghorman`) — steps TBD (D10)
2. **R4 base link:** plug the UNO R4 WiFi into a Pi USB port; udev rule `/etc/udev/rules.d/99-scout-base.rules` → `/dev/scout_base` (USB ID `2341:1002`). TBD: final rule text.
3. **Flash firmware:** `arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi firmware/r4_base`, then `arduino-cli upload -p /dev/scout_base --fqbn arduino:renesas_uno:unor4wifi firmware/r4_base` (or the Arduino IDE on the Mac).
4. **Bring-up:** `ros2 launch scout_bringup robot.launch.py` — TBD
5. **Recover:** USB serial drop / R4 reset, DDS discovery issues, undervoltage (`vcgencmd get_throttled`) — TBD
