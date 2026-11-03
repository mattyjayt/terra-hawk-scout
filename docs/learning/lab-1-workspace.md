# Lab 1 — Workspace, packages, VS Code, udev
**Objectives:** a ROS 2 workspace inside the repo that builds; the first two packages; a stable device name for the R4.
**Reading:** ROS 2 tutorials "Creating a workspace" and "Creating a package" (Beginner: Client Libraries).

## Background: what a workspace is
A ROS 2 *workspace* is a folder with a `src/` of packages. `colcon build` builds every package into `install/`, and `source install/setup.bash` overlays them on top of `/opt/ros/jazzy` (the *underlay*). `--symlink-install` links Python files and launch files instead of copying them, so edits take effect without a rebuild (C++ still needs one).

---

## Task 1.1 — VS Code Remote-SSH and the shell
**Instructions**
1. VS Code on the Mac → *Remote-SSH: Connect to Host…* → `ghorman` → open the folder `~/Projects/terra-hawk-scout`.
2. Install these extensions **on the remote**: Python (Microsoft), C/C++ (Microsoft) or clangd, CMake Tools. Optional: the ROS 2 extension ("Robot Developer Extensions for ROS 2").
3. Add to `~/.bashrc` on the Pi (once):
   ```bash
   source /opt/ros/jazzy/setup.bash
   export ROS_DOMAIN_ID=0          # same on every machine that should see the Scout
   alias sws='source ~/Projects/terra-hawk-scout/ros2_ws/install/setup.bash'
   ```
4. Check `.gitignore` at the repo root has `ros2_ws/build/`, `ros2_ws/install/`, `ros2_ws/log/` and `bags/`. Add any that are missing.

**Check:** a new VS Code terminal prints nothing red; `echo $ROS_DISTRO` → `jazzy`; `git status` stays clean after a build.

## Task 1.2 — Create `scout_base_py` 🧠
**Instructions:** in `ros2_ws/src`:
```bash
ros2 pkg create scout_base_py --build-type ament_python --license Apache-2.0 \
  --maintainer-name "Jaime Tellie" --maintainer-email jaime.tellie@gmail.com \
  --description "Scout base bridge (Python): ROS 2 <-> UNO R4 over USB serial" \
  --dependencies rclpy geometry_msgs sensor_msgs std_msgs std_srvs diagnostic_msgs \
  --node-name base_bridge
```
Then:
1. In `package.xml` add the system dependencies that aren't ROS packages:
   ```xml
   <exec_depend>python3-serial</exec_depend>
   <test_depend>python3-pytest</test_depend>
   ```
2. Create empty modules with a one-line docstring each: `scout_base_py/protocol.py`, `kinematics.py`, `conversions.py`. You fill them in Lab 3. `base_bridge.py` already exists (the generated "Hi from" node).
3. Look at `setup.py`: find the `entry_points` line that turns `base_bridge` into an executable.

**Check**
```bash
cd ~/Projects/terra-hawk-scout/ros2_ws
colcon build --symlink-install && sws
ros2 run scout_base_py base_bridge          # → "Hi from scout_base_py."
colcon test --packages-select scout_base_py && colcon test-result --verbose
```
All generated tests (flake8, pep257) pass; the copyright test is skipped.

## Task 1.3 — Create `scout_bringup` (launch + config only) 🔧/🧠
**Instructions**
```bash
ros2 pkg create scout_bringup --build-type ament_cmake --license Apache-2.0 \
  --maintainer-name "Jaime Tellie" --maintainer-email jaime.tellie@gmail.com \
  --description "Scout launch files, parameters and Foxglove layouts"
```
Delete `include/` and `src/`, and create `launch/`, `config/` and `foxglove/`. In `CMakeLists.txt`, before `ament_package()`:
```cmake
install(DIRECTORY launch config foxglove DESTINATION share/${PROJECT_NAME})
```
Add `<exec_depend>launch_ros</exec_depend>` and `<exec_depend>foxglove_bridge</exec_depend>` to `package.xml`. Empty folders don't install, so put a `.gitkeep` in each.

**Check:** `colcon build` → 2 packages finished; `ros2 pkg prefix scout_bringup` prints a path.

## Task 1.4 — A stable name for the R4: udev rule 🧠
**Background:** `/dev/ttyACM0` is handed out in plug-in order: plug in a LiDAR first and the R4 becomes `ttyACM1`. A udev rule matches the device's USB identity and creates a fixed symlink.

**Instructions**
1. Find the identity: `udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct|serial' | head`
2. Write `tools/udev/99-scout-base.rules` (in the repo, so it's documented) with **one** rule that:
   - matches `SUBSYSTEM=="tty"`, `ATTRS{idVendor}`, `ATTRS{idProduct}` and `ATTRS{serial}`
   - sets `SYMLINK+="scout_base"`, `GROUP="dialout"`, `MODE="0660"`
   - sets `ENV{ID_MM_DEVICE_IGNORE}="1"`, so ModemManager doesn't probe the port and send it AT commands
3. Install it:
   ```bash
   sudo cp tools/udev/99-scout-base.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

**Check:** `ls -l /dev/scout_base` → `-> ttyACM0`. Unplug and replug the R4: the link comes back. `python3 -m serial.tools.miniterm /dev/scout_base 115200` shows `I` lines (Ctrl-] quits).

## Questions (lab report)
1. What is the difference between an *underlay* and an *overlay*? What happens if you forget `source install/setup.bash` in a new terminal?
2. Why does `--symlink-install` help for Python but not for C++?
3. Why match on the serial number and not only on vendor/product ID? Think of the Garden's R4.
