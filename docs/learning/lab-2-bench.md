# Lab 2 — Bench characterisation
**Objectives:** know the hardware before you model it. Every number the bridge needs (`track_width`, `max_wheel_speed`, `pwm_min`, `pan_sign`, IMU axes, noise) comes from this lab, not from a guess.
**Tools:** `python3 -m serial.tools.miniterm /dev/scout_base 115200` (type commands, Ctrl-] quits) or `tools/serial_teleop.py`, a tape measure, a multimeter, a phone camera.
**Reading:** [serial-r4-v1](../specs/interfaces/serial-r4-v1.md) · REP-103 · REP-145 · DD-SCOUT-BRIDGE §5.5, §6.

> Record every measurement in the lab report **and** in `ros2_ws/src/scout_bringup/config/base.yaml` (Task 2.6).

---

## Task 2.1 — The frozen IMU (fault investigation) 🧠
**Observed by Argos, 3 Nov ~11:30:** 120 consecutive `I` lines with *identical* IMU values (`0.036 0.003 0.827 0.01 0.04 0.06`), while `t_ms` kept counting. Opening the port did **not** reset the R4. Battery read `0.00` V.

**Background:** a real MEMS sensor always has noise in the last printed digit. Identical values mean `readImu()` is failing, so `updateImu()` returns early and `sendTelemetry()` re-sends the old globals. The firmware never says so.

**Instructions**
1. Confirm it: watch 5 s of `I` lines in miniterm. Frozen or alive?
2. Power-cycle the R4 (unplug USB, battery off, wait 5 s, plug in). Watch the boot lines. Is the IMU alive now?
3. Try to make it freeze again and note what triggers it:
   - (a) just wait 10 min
   - (b) drive the motors (battery on, wheels up) for 30 s
   - (c) wiggle the shield and IMU cable
4. Write down your hypotheses and the evidence for or against each:
   - I²C bus lock-up after electrical noise from the motors
   - the MPU-6050 dropping into sleep or resetting (brown-out)
   - a loose connector
   - a firmware bug (look at `readImu()`: what does it do when `Wire.requestFrom` returns fewer bytes?)
5. Don't fix the firmware now: that's issue #16. Write a finding for DD-SCOUT-FW: what v2 must do when the IMU read fails (report it, retry, re-initialise the bus?).

**Check:** you can reproduce the freeze (or show it doesn't come back), and you have a ranked list of causes. The bridge's stale detector (Lab 3, U7) exists because of this task.

## Task 2.2 — IMU axes and signs 🧠
**Instructions:** car on a flat table, IMU alive. For each pose, note the sign and approximate value of `ax ay az` (in g):
| Pose | ax | ay | az |
|---|---|---|---|
| flat, wheels down | | | |
| nose down (front on the table, standing on its nose) | | | |
| left side down | | | |
Then, flat again, turn the car **left** (counter-clockwise seen from above) by hand and note the sign of `gz`, plus the change in `yaw`.

**Work out:**
- Which chip axis points forward, left and up? An accelerometer at rest measures **+1 g along the axis that points up** (specific force, REP-145).
- The rotation from `base_link` to `imu_link` (roll, pitch, yaw), for the URDF in Lab 6.
- **The puzzle:** the firmware has `YAW_SIGN = −1` ("gave +90 for a left turn"), and the frozen sample had `az = +0.83`. For a rigid chip with z up, a left turn gives **+gz**. Can both facts be true? If not, which is wrong? (A chip mounted upside down reads az ≈ −1 g.) Is 0.83 g instead of 1.0 g a clue too?

**Check:** a table of chip axis → robot axis that explains every row above without contradictions.

## Task 2.3 — Pan servo 🧠
**Instructions:** send `V 90`, `V 45`, `V 135`, `V 0`, `V 180`, then `V 90`.
- Which way does the camera turn for `V 135`? REP-103 says + = left, so set `pan_sign = +1` if `V 135` turns left, else −1.
- Do 0 and 180 really give ±90°? Many SG90s only do about ±80°. Listen for buzzing at the ends: that's the servo stalling against its stop. Pick safe `pan_min/pan_max`.
- Does the camera ribbon cable limit the travel?

**Check:** `pan_sign`, measured angle at 45/135 (phone protractor app or a printed protractor), safe limits.

## Task 2.4 — Battery reading
**Instructions:** battery switch on. Compare the `batt` field with a multimeter across the pack terminals, 3 times (idle, motors at `M 150 150` wheels up, idle again).
**Check:** error ≤ 0.1 V, else note the correction factor (the firmware already multiplies by 1.08). Which pack is it (2S Elegoo box → `cell_count 2`)?

## Task 2.5 — Drive calibration (wheels **on the floor**) 🧠
Use **timed** commands. They stop by themselves and are acknowledged (`A M ok`, then `W stop`).
1. **`pwm_min`:** `M 40 40 1000`, `M 45 45 1000`, … in steps of 5 until both sides turn reliably from standstill. Do it on the hard floor and on a carpet tile. Use the higher value.
2. **`max_wheel_speed`:** mark the start, `M 255 255 2000`, measure the distance d (m). v_max = d / 2.0. Repeat 3× and average. Does it drive straight? If not, which side is slower? (That's open-loop asymmetry: write it down; the EKF and LiDAR handle it later.)
3. **Linearity:** repeat at PWM 150 and 200. Plot distance against PWM (3 points + pwm_min). Is speed roughly linear in PWM above the deadband? DD-SCOUT-BRIDGE §5.3 *assumes* it is.
4. **`track_width`:** centre of the left tyres to centre of the right tyres (m).

**Check:** `pwm_min`, `max_wheel_speed`, `track_width`, plus a sentence on straightness and linearity.

## Task 2.6 — Parameter file 🔧/🧠
Create `ros2_ws/src/scout_bringup/config/base.yaml`. The top-level key is the **node name**:
```yaml
base_bridge:
  ros__parameters:
    port: /dev/scout_base
    track_width: 0.0        # Task 2.5
    max_wheel_speed: 0.0    # Task 2.5
    pwm_min: 0              # Task 2.5
    pan_sign: 1             # Task 2.3
    pan_min: -1.5708
    pan_max: 1.5708
    cell_count: 2           # Task 2.4
    # the rest: defaults from IF-ROS-TOPICS §8
```
**Check:** every value has a comment naming the task that measured it.

## Questions (lab report)
1. Why is silently repeating the last IMU sample more dangerous for an EKF than sending nothing?
2. What does `pwm_min` do to the robot's ability to drive very slowly, and why does that matter for Nav2 later?
3. If the left side is 10 % slower than the right, what does the robot do when you command `(v = 0.2, ω = 0)`? Where in the system should that be corrected: firmware, bridge or a higher layer?
