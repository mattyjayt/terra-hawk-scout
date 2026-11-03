# Lab 6 — Robot model: URDF, TF, bringup launch
**Objectives:** describe the Scout's frames in a URDF so that TF knows where the IMU and the camera are, including the moving pan joint; one launch command brings the base up.
**Reading:** ROS 2 tutorials "URDF" (Building a visual robot model, Using xacro), "tf2" introduction; REP-105; IF-ROS-TOPICS §7; Articulated Robotics, "Describing a robot with URDF" (YouTube).

## Background
`robot_state_publisher` reads the URDF from the `robot_description` parameter and publishes:
- **fixed** joints once on `/tf_static`
- **moving** joints on `/tf`, each time a `/joint_states` message arrives

Your bridge already publishes `pan_joint`, so the camera frame will follow the servo without any extra code.

---

## Task 6.1 — `scout_description` package 🧠
```bash
sudo apt install ros-jazzy-xacro
ros2 pkg create scout_description --build-type ament_cmake --license Apache-2.0 \
  --maintainer-name "Jaime Tellie" --maintainer-email jaime.tellie@gmail.com
```
Delete `include/` and `src/`, and add `urdf/` and `launch/`. Install both folders (same pattern as Lab 1.3). Add `<exec_depend>xacro</exec_depend>`, `<exec_depend>robot_state_publisher</exec_depend>`.

## Task 6.2 — `urdf/scout.urdf.xacro` 🧠
The tree is in IF-ROS-TOPICS §7:
- `base_link` (floor, chassis centre)
- `imu_link` (fixed; rotation from Lab 2.2)
- `pan_base_link` (fixed, servo position)
- `pan_link` via a revolute `pan_joint` about z, limits from Lab 2.3
- `camera_link` (fixed)
- `camera_optical_frame` (fixed, `rpy="${-pi/2} 0 ${-pi/2}"`)

**Rules**
- Every offset is a xacro property at the top of the file, with a comment naming its source (`<!-- measured 2026-11-04, tape -->` or `<!-- cad/scout-v2/out/robot_params.json -->`).
- Geometry: a simple box for the chassis (Elegoo plate 0.2308 × 0.1325 m), cylinders for the wheels (r = 0.0325 m), a small box for the camera. The visuals only have to be recognisable.
- Measure the IMU and camera positions on *this* robot (the Elegoo chassis today, not the v2 CAD).

**Check:** `xacro scout.urdf.xacro > /tmp/scout.urdf && check_urdf /tmp/scout.urdf` prints the tree with (at least) these 6 links.

## Task 6.3 — Launch files 🧠
1. `scout_description/launch/description.launch.py`: `robot_state_publisher` with
   ```python
   robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
   ```
2. `scout_bringup/launch/base.launch.py` with these arguments:
   - `impl` (`py`|`cpp`, default `py`)
   - `port` (default `/dev/scout_base`)
   - `foxglove` (`true`|`false`)

   It starts: the bridge from `scout_base_<impl>` with `config/base.yaml`; it includes `description.launch.py`; and, if `foxglove` is true, `foxglove_bridge`.

**Check:** `ros2 launch scout_bringup base.launch.py foxglove:=true` brings everything up with **one** command (REQ-SCOUT-016). Then:
- `ros2 run tf2_tools view_frames` → a PDF with the full tree
- the Foxglove 3D panel shows the model
- publishing `/pan/cmd` 0.5 visibly turns `camera_link` left

## Questions (lab report)
1. Why is the IMU's mounting rotation in the URDF and not in the bridge code?
2. What does `camera_optical_frame` exist for, when `camera_link` already exists? (Hint: image x/y/z conventions.)
3. `pan_joint` is the *commanded* angle. When does the TF of the camera lie, and by how much (servo speed ≈ 0.1 s / 60°)?
