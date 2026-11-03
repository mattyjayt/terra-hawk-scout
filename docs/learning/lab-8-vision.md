# Lab 8 — Vision: detect a person, keep them in view with the pan servo
**Objectives:**
- Camera frames in ROS.
- A pretrained YOLO11n detector, exported to NCNN, running on the Pi 5.
- A tracker node that turns the camera toward the person and uses the gyro so the robot's own rotation doesn't throw it off.

The vision code stays **Python only** (industry practice for this layer).
**Reading:** Ultralytics Raspberry Pi guide, NCNN integration, Track mode (links in the README) · ByteTrack (arXiv:2110.06864) · vision_msgs (https://github.com/ros-perception/vision_msgs) · ROS 2 how-to "Using Python packages with ROS 2" (virtual environments) · IF-ROS-TOPICS T10–T14.

## Background
- **Detection ≠ recognition.** COCO-pretrained YOLO already has class 0 = `person`, so no training is needed to follow *a* person. Recognising *you* would be a separate identity problem (face or re-ID), and it raises the question of what the robot does with everyone else.
- **Tracking** gives each person a stable ID across frames (ByteTrack: a Kalman-filter motion model + IoU matching), so the servo follows *one* person instead of jumping between people.
- **Visual servoing:** the error is measured in the image (pixels from the centre) and corrected by moving the camera. **Fusion with the gyro:** the camera gives a slow, delayed but *absolute* bearing; the gyro gives a fast *relative* rate. Counter-rotating the pan by the base's measured yaw rate is a feed-forward / complementary-filter idea, and it keeps the target in view while the robot turns.

---

## Task 8.0 — Design doc first 🧠
Write `docs/design/DD-SCOUT-PERC.md` (sections: context and requirements REQ-SCOUT-010/023, goals/non-goals, nodes and topics (from IF-ROS-TOPICS P2), detector design, tracker control law, test plan, risks). Argos reviews; no code until it's approved.

## Task 8.1 — Camera into ROS 🔧 (Argos), known blocker
**Status (3 Nov):**
- `rpicam-still` works.
- The apt `camera_ros` reports *"no cameras available"*, because `ros-jazzy-libcamera` (0.7.2) has no Pi 5 PiSP pipeline.
- A source build of `camera_ros` (`~/camera_ws`) against the Pi's `libcamera-rpi` (0.7.1) does find the imx708, but the IPA helper process crashes ("A list of V4L2 controls requires a ControlInfoMap"), probably because two libcamera builds are mixed.

**Next options, in order:**
1. `sudo apt remove ros-jazzy-camera-ros ros-jazzy-libcamera`, so only the Pi's libcamera is left, then rebuild `camera_ros` from source.
2. Pin `camera_ros` to a release that matches libcamera 0.7.1.
3. Fallback: a tiny publisher fed by `rpicam-vid --codec yuv420 -o -`, or GStreamer `libcamerasrc`.

Parameters once it works: `-p camera:=0 -p width:=640 -p height:=480 -p format:=RGB888 -p frame_id:=camera_optical_frame`.
**Check:** `ros2 topic hz /camera/image_raw` ≥ 15; `/camera/image_raw/compressed` is visible in a Foxglove Image panel. Note the `encoding` field.

## Task 8.2 — Python environment for Ultralytics 🔧/🧠
```bash
python3 -m venv --system-site-packages ~/venvs/scout-vision   # sees rclpy, cv_bridge from /opt/ros
source ~/venvs/scout-vision/bin/activate
pip install "ultralytics" "numpy<2"        # numpy 2 breaks cv_bridge built against 1.26
python -c "import numpy, cv2, rclpy, ultralytics; print(numpy.__version__, cv2.__file__)"
```
For colcon to use the venv's Python in your package's scripts, add to `setup.cfg`:
```ini
[build_scripts]
executable = /usr/bin/env python3
```
and always `source ~/venvs/scout-vision/bin/activate` **before** `colcon build` and before `ros2 run`.
**Check:** the one-liner prints numpy 1.26.x, and all four imports work.

## Task 8.3 — Export and benchmark the model 🧠
```bash
yolo export model=yolo11n.pt format=ncnn imgsz=320     # → yolo11n_ncnn_model/
yolo predict model=yolo11n_ncnn_model source=/tmp/t.jpg imgsz=320
```
Measure the inference time (Ultralytics prints it) for `imgsz` 320, 480 and 640, with the PyTorch model and the NCNN model. Make a table. Pick the smallest `imgsz` that still detects you reliably at 3 m.
**Check:** the table, and a choice with a reason. The target for REQ-SCOUT-010 is ≥ 10 FPS end to end.

## Task 8.4 — Package `scout_perception` and `person_detector` 🧠
```bash
ros2 pkg create scout_perception --build-type ament_python --license Apache-2.0 \
  --maintainer-name "Jaime Tellie" --maintainer-email jaime.tellie@gmail.com \
  --dependencies rclpy sensor_msgs vision_msgs std_msgs cv_bridge geometry_msgs \
  --node-name person_detector
```
(Your plan called it "vision". `scout_perception` matches the repo layout in the README.)
- Subscribe to `camera/image_raw` with **best effort, depth 1**. If inference is slower than the camera, you always get the newest frame and old ones are dropped. That's what you want for control.
- `cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')`
- `results = model.track(frame, persist=True, classes=[0], conf=conf_threshold, imgsz=imgsz, tracker='bytetrack.yaml', verbose=False)`
- Fill `Detection2DArray` **exactly** as IF-ROS-TOPICS T13: copy the header from the image, centre/size in original pixels, `class_id='person'`, `score`, `id=str(track_id)`. Publish an **empty** array when nobody is there.
- Optional debug image `detections/image/compressed` at ≤ 5 Hz (boxes drawn with `results[0].plot()`, JPEG via `cv2.imencode`).
- Parameters: `model_path`, `imgsz`, `conf_threshold`, `debug_image`.

**Check:** `ros2 topic hz /detections` ≥ 10; `ros2 topic echo --once /detections` with you in view shows one detection, a score > 0.5 and a stable `id`.

## Task 8.5 — `pan_tracker`: the control law, test first 🧠
Pure functions in `scout_perception/tracking.py`, with tests in `test/test_tracking.py`:
```python
def select_target(dets, last_id) -> det | None        # keep last_id if still present, else highest score
def bearing_error(cx, cx0, fx) -> float               # rad, = atan((cx - cx0) / fx); + when the person is RIGHT of centre
def next_pan(pan_now, err, gyro_z_base, dt, p) -> float
    # pan_now - kp*err  (person right -> pan right = negative, REP-103)
    # - gyro_z_base*dt   (feed-forward: base turns left -> pan right by the same amount)
    # deadband |err| < 2 deg -> no correction from err; rate limit; clamp to pan limits
```
- `fx`: from `camera/camera_info` (`K[0]`) when calibrated, else `fx = (W/2) / tan(HFOV/2)` (Camera Module 3: HFOV ≈ 66°, wide version ≈ 102°: check which you have).
- Node `pan_tracker`:
  - subscribes `detections`, `joint_states` (current `pan_joint`), `imu/data_raw`
  - rotates the IMU's angular velocity into `base_link` with tf2 (the URDF from Lab 6 knows the mounting)
  - publishes `pan/cmd` at 20 Hz
  - when the target is lost: hold for `lost_hold_s` (1 s), then return to 0 slowly
- **Tests first:**
  - the sign of `bearing_error` for a person left/right of centre
  - `next_pan` moves toward the person
  - feed-forward cancels a pure base rotation (err = 0, gyro = +0.5 rad/s → pan decreases by 0.5·dt)
  - deadband, rate limit, clamp
  - `select_target` keeps the ID when a higher-scoring stranger appears

**Check (SIL, no camera):** a small script publishes fake `Detection2DArray` with `cx` moving in a sine wave. `pan/cmd` follows with the right sign, and `base_bridge` (fake R4) receives `V` lines.

## Task 8.6 — HITL: follow a person (TC-SCOUT-025) 🧠
Robot on the floor, wheels **not** driven (only the pan servo moves). Start `base.launch.py`, `person_detector`, `pan_tracker` and Foxglove. Walk slowly (≤ 0.5 m/s) across the view for 20 s while recording a bag: `/detections`, `/pan/cmd`, `/joint_states`, `/imu/data_raw`.
- Metric: the fraction of frames with a detection where |cx − cx0| < 0.1·W. Pass ≥ 80 % (REQ-SCOUT-023). Compute it from the bag with a short script.
- Then turn the **robot** slowly by hand while you stand still, once with feed-forward on and once with it off (parameter `use_gyro_ff`). Compare the pixel error.

**Privacy:** camera bags contain people. Record only yourself and don't commit bags or frames. The video goes to Drive.
**Evidence:** a screen recording of the Foxglove Image panel with boxes + pan plot, next to a phone video of the servo following you.

## Questions (lab report)
1. Why best-effort, depth 1 for the image subscription in a control loop? What would depth 10, reliable do to the latency?
2. Each detection is ~100 ms old when it arrives. Using `joint_states` at *now* instead of at the image's time makes the controller over- or under-shoot. Which one, and how would you fix it? (A pan history buffer, looked up by the image stamp.)
3. Gyro feed-forward vs. only visual feedback: what does each one do well, and what goes wrong with each one alone?
4. A robot that recognises one specific person also notices everyone who isn't that person. Where should TerraHawk draw the line, and how would you build that line into the system (data retention, on-device only, opt-in)?
