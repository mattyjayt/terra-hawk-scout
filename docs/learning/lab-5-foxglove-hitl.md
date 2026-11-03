# Lab 5 — Foxglove, teleop, hardware in the loop, rosbag
**Objectives:** see the robot's data live on the Mac, drive it from a keyboard and from Foxglove, run the contract tests against the **real** R4, and record evidence (bags, screen recording).
**Reading:** Foxglove ROS 2 docs (https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2) · DD-SCOUT-BRIDGE §8.3 · ROS 2 tutorial "Recording and playing back data".

## Background: why Foxglove and not RViz
The Pi runs headless. `foxglove_bridge` on the Pi serves every topic over one WebSocket, and the Foxglove app on the Mac renders it. No DDS traffic crosses Wi-Fi, so there's no discovery trouble. The same layout also opens recorded bags, which is how you'll debug later.

---

## Task 5.1 — Foxglove bridge and connection
**Instructions**
1. Pi: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765`
2. Mac: Foxglove app → *Open connection* → *Foxglove WebSocket* → `ws://ghorman.local:8765` (fallback `ws://192.168.178.147:8765`).
3. Start the bridge on the **fake** R4 first (Lab 4), then on `/dev/scout_base`.

**Check:** the Topics list shows every P1 topic with the right type.

## Task 5.2 — A layout you'll reuse 🧠
Build one layout, *Scout P1*:
| Panel | Content |
|---|---|
| Plot | `/imu/data_raw.angular_velocity.z` and `/base/debug/yaw.data` (two Y axes) |
| Plot | `/imu/data_raw.linear_acceleration.{x,y,z}` |
| Raw Messages | `/diagnostics` |
| Gauge or Indicator | `/battery.voltage` (red below the pack's safe voltage, e.g. 6.6 V for 2S) |
| Teleop | publishes to `/cmd_vel`. Check the message type it uses in the panel settings: the contract says `geometry_msgs/msg/Twist`. Keep the speeds small (0.15 m/s, 0.8 rad/s). |
| Publish | `/pan/cmd`, `std_msgs/msg/Float64`, with `{"data": 0.5}` and `{"data": -0.5}` presets |
| 3D | added in Lab 6 (TF + model) |
Export the layout (*Layout → Export…*) to `ros2_ws/src/scout_bringup/foxglove/scout_p1.json` and commit it.

**Check:** tilting the car moves the accel plot; turning it by hand moves `angular_velocity.z` and the debug yaw together.

## Task 5.3 — Hardware in the loop (HITL) 🧠
**Safety first:** wheels **off the ground** on a box, battery on, switch within reach.
1. Bridge on `/dev/scout_base` with your `base.yaml`.
2. Keyboard teleop in a second terminal: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`. Its defaults (0.5 m/s, 1.0 rad/s) exceed `max_linear`, so the bridge clamps. Press `z`/`x` to scale down.
3. Run the DD-SCOUT-BRIDGE §8.3 table and fill in pass/fail with numbers:
   - TC-SCOUT-001: forward → both sides forward; `j` (left turn) → right side forward, left back
   - TC-SCOUT-002: drive, **pull the USB cable**, film at 60 fps, count the frames until the wheels stop (< 300 ms = 18 frames)
   - TC-SCOUT-005/022: `ros2 service call /imu/calibrate std_srvs/srv/Trigger`, then 30 s still: gyro mean < 0.01 rad/s, |a| = 9.81 ± 0.3, `imu_frozen: false`
   - TC-SCOUT-013: `/battery.voltage` vs multimeter
   - TC-SCOUT-023: `/pan/cmd` +0.5 → camera turns **left**
4. Run the **automated** contract test against the hardware. The fake-R4 tests (kill/restart) don't apply; add an env var such as `SCOUT_HITL=1` that skips them and points `PORT` at `/dev/scout_base`.

**Check:** the TP-SCOUT rows TC-SCOUT-001/002/005/013/021–024 have a date, commit and result.

## Task 5.4 — Record evidence with rosbag
```bash
mkdir -p ~/Projects/terra-hawk-scout/bags     # gitignored
ros2 bag record -o bags/hitl_$(date +%F_%H%M) /imu/data_raw /battery /joint_states /diagnostics /cmd_vel /base/debug/yaw
```
1. A 30 s still bag → compute the gyro and accel **variances** per axis (a short Python script with `rosbag2_py`, or Foxglove's plot statistics) → put them into `base.yaml` as `gyro_var`/`accel_var` (DD-SCOUT-BRIDGE §6.4).
2. A 60 s teleop bag → open it in Foxglove with your layout (*Open local file*).

**Check:** the bag replays in Foxglove (TC-SCOUT-017), and `base.yaml` has measured covariances with a comment saying which bag they came from.

## Task 5.5 — Screen recording for the portfolio
Record the Mac screen (Cmd-Shift-5) with the phone filming the robot at the same time. Shot list (30–60 s total):
1. Foxglove layout full screen; tilt the car → the accel plot reacts (5 s).
2. Teleop panel → wheels spin (wheels-up, then on the floor) (10 s).
3. `/pan/cmd` ±0.5 → camera pans; `joint_states` follows (5 s).
4. Pull the USB cable → wheels stop; `/diagnostics` turns red, then green when it's plugged back (10 s).
5. Terminal: `colcon test-result` all green (5 s).
Files go to Drive `TerraHawk/Media/S2/` (no need to film yourself coding).

## Questions (lab report)
1. Why does `foxglove_bridge` avoid the Wi-Fi DDS discovery problems that RViz on the Mac would have?
2. Your still-bag variance: how does it compare to the MPU-6050 datasheet noise density (gyro 0.005 °/s/√Hz)? What bandwidth does that imply?
3. In the USB-pull test, which safety layer stopped the wheels: the bridge, the R4 watchdog, or both? How do you know?
