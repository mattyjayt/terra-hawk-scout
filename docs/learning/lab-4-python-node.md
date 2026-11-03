# Lab 4 — Python bridge node + software-in-the-loop tests
**Objectives:** wrap the Lab 3 core in a ROS 2 node that implements IF-ROS-TOPICS P1 completely, and prove it against the fake R4 with an automated launch test.
**Reading:** ROS 2 tutorials "Writing a simple publisher and subscriber (Python)", "Using parameters in a class (Python)", "Writing a simple service and client (Python)"; Concepts "About QoS settings" and "Executors"; DD-SCOUT-BRIDGE §3–§7; the launch_testing README (https://github.com/ros2/launch/tree/jazzy/launch_testing).

## Background
- **Executor and callbacks:** `rclpy.spin(node)` runs your subscription, timer and service callbacks one at a time (single-threaded executor). A callback that blocks stops every other callback. That's why the serial *reader* is a separate thread (DD-SCOUT-BRIDGE §3).
- **Threads in Python:** the reader thread and the callbacks share state, such as the last ack, the link state and the serial port for writing. Protect writes with a `threading.Lock`, and signal acks with a `threading.Event`. The GIL protects the interpreter, not your logic.
- **QoS:** a best-effort subscriber can read a reliable publisher, but not the reverse. Use the profiles from IF-ROS-TOPICS §3.

---

## Task 4.1 — Node skeleton: parameters, publishers, subscribers 🧠
Replace the generated `base_bridge.py` with a class `BaseBridge(Node)`, node name **`base_bridge`**.
1. Declare **every** parameter in IF-ROS-TOPICS §8, with its type and default (`self.declare_parameter('track_width', 0.0)`). Refuse to start with a clear error if `track_width`, `max_wheel_speed` or `pwm_min` is still 0.
2. Publishers, with names **without a leading slash**:
   | name | type | QoS |
   |---|---|---|
   | `imu/data_raw` | `sensor_msgs.msg.Imu` | `qos_profile_sensor_data` |
   | `battery` | `sensor_msgs.msg.BatteryState` | 10 (reliable) |
   | `joint_states` | `sensor_msgs.msg.JointState` | 10 |
   | `diagnostics` | `diagnostic_msgs.msg.DiagnosticArray` | 10 |
   | `base/debug/yaw` | `std_msgs.msg.Float64` | `qos_profile_sensor_data` |
3. Subscriptions: `cmd_vel` (`geometry_msgs.msg.Twist`, 10) and `pan/cmd` (`std_msgs.msg.Float64`, 10). For now the callbacks only **store** the value and the time (`self.get_clock().now()`).
4. Timers: `send` at `send_rate` and `diagnostics` at 1 Hz (empty bodies for now).

**Check:** `ros2 run scout_base_py base_bridge --ros-args --params-file <config/base.yaml>`, then `ros2 node info /base_bridge` lists exactly the topics above, and `ros2 param dump /base_bridge` shows your values.

## Task 4.2 — Serial link, reader thread, state machine 🧠
1. `open_port()`: `serial.Serial(port, baud, timeout=0.1)`, then `reset_input_buffer()`, then write `S\n` (DD-SCOUT-BRIDGE §4).
2. Reader thread (`daemon=True`), loop while `self._running`:
   - not connected → try `open_port()` every 1 s and catch `serial.SerialException` / `OSError`
   - connected → `readline()`, decode with `errors="replace"`, `parse_line()`, then dispatch: `ImuSample` → publish (Task 4.4); `Ack` → store the text, `self._ack_event.set()`; `WatchdogStop` → counter++
   - any exception from the port → close, state = DISCONNECTED, log **one** warning (not one per loop)
3. `write_line(text)`: under `self._write_lock`; if not connected, do nothing and return False.
4. States DISCONNECTED → SYNCING → RUNNING exactly as in DD-SCOUT-BRIDGE §4. Log each transition once.

**Check:** with no device at the port, the node runs and logs "waiting for /tmp/fake_r4". Start `python3 tools/fake_r4.py` and within 1 s the log shows SYNCING → RUNNING.

## Task 4.3 — Drive: `cmd_vel` → `M l r`, with the timeout 🧠
In the `send` timer:
- the command is fresh (age < `cmd_timeout`) → `twist_to_pwm(...)` → `write_line(format_drive(l, r))`
- it has just gone stale → write `S\n` **once**, then stay quiet

Clamp `v` and `ω` to `max_linear`/`max_angular` first.

**Check (fake R4):**
```bash
# T1: python3 tools/fake_r4.py --log /tmp/r4.jsonl
# T2: ros2 run scout_base_py base_bridge --ros-args --params-file .../base.yaml -p port:=/tmp/fake_r4
# T3: ros2 topic pub -r 10 -t 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
# T4: tail -f /tmp/r4.jsonl
```
You see `M l l` lines at about 20 Hz with the value you computed by hand, then one `S` about 0.25 s after T3 ends.

## Task 4.4 — Telemetry: IMU, battery, joint state, debug yaw 🧠
For every `ImuSample`:
- `Imu`: field by field from IF-ROS-TOPICS T3 (stamp = now, `frame_id`, `orientation_covariance[0] = -1.0`, unit conversions, covariance diagonals). Then feed the `StaleDetector`.
- `JointState`: `name=['pan_joint']`, `position=[servo_deg_to_pan_rad(...)]`.
- `base/debug/yaw`: radians.
- Battery: append to a deque of 20 samples; a 1 Hz timer publishes `BatteryState` (T4). Use `float('nan')` for unknown fields.

**Check:** `ros2 topic hz /imu/data_raw` ≈ 20; `ros2 topic echo --once /imu/data_raw` shows `frame_id: imu_link`, `orientation_covariance[0] = -1.0` and `linear_acceleration.z ≈ 9.81` (fake). `ros2 topic echo /battery` → 7.6 V, `present: true`.

## Task 4.5 — Pan command and services 🧠
- `pan/cmd` → `pan_rad_to_servo_deg` → `V` only if it changed by ≥ 1°, at most 20 Hz.
- Services `base/stop`, `imu/calibrate`, `imu/zero_yaw` (`std_srvs.srv.Trigger`): clear the event, write the command, `wait(timeout)`, then fill `success` and `message` (IF-ROS-TOPICS §6). This only works because the reader runs in its own thread: try it in the main thread once and watch it hang. That's the lesson.

**Check:** `ros2 topic pub --once /pan/cmd std_msgs/msg/Float64 "{data: 0.5}"` → the fake log shows `V 119`; `ros2 service call /base/stop std_srvs/srv/Trigger` → `success=True`; with the fake stopped → `success=False, message='link down'`.

## Task 4.6 — Diagnostics and shutdown 🧠
- The 1 Hz timer builds one `DiagnosticStatus` (name `base_bridge: R4 link`, `level` from `link_level()`, `values` from IF-ROS-TOPICS T6) as `KeyValue` pairs (strings!). In Python, `level` is a `bytes` field: `DiagnosticStatus.OK` etc. are already the right type.
- Shutdown in `main()`:
  ```python
  from rclpy.executors import ExternalShutdownException
  try:
      rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
      pass
  finally:
      node.close()          # your method: write 'S', stop the thread, join it, close the port
      node.destroy_node()
      rclpy.try_shutdown()
  ```

**Check:** stop the fake → `/diagnostics` goes to level 2 within 1 s → restart it → back to 0 within 3 s. Run the fake with `--freeze-imu` → `imu_frozen: true`, level 2. Ctrl-C the bridge → the last line in the fake log is `S`.

## Task 4.7 — Automated SIL contract test (launch_testing) 🔧 + 🧠
The harness is boilerplate (🔧, given below); the test cases are yours (🧠). It lives in `scout_bringup/test/test_base_contract.py`, so the **same file** later tests the C++ bridge (`SCOUT_BRIDGE_IMPL=cpp`).

`scout_bringup/package.xml`: `<test_depend>launch_testing_ament_cmake</test_depend>`, `<test_depend>launch_testing_ros</test_depend>`, `<test_depend>python3-pytest</test_depend>`.
`scout_bringup/CMakeLists.txt`:
```cmake
if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_base_contract.py TIMEOUT 120)
endif()
```
Harness (the fake R4 is owned by the test, so tests can kill and restart it):
```python
import json, os, subprocess, sys, time, unittest
import launch, launch_ros.actions, launch_testing.actions, launch_testing.asserts
import pytest, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

REPO = os.environ.get('SCOUT_REPO', os.path.expanduser('~/Projects/terra-hawk-scout'))
FAKE = os.path.join(REPO, 'tools', 'fake_r4.py')
PORT, LOG = '/tmp/fake_r4_test', '/tmp/fake_r4_test.jsonl'
PKG = {'py': 'scout_base_py', 'cpp': 'scout_base_cpp'}[os.environ.get('SCOUT_BRIDGE_IMPL', 'py')]
PARAMS = {'port': PORT, 'track_width': 0.15, 'max_wheel_speed': 0.5, 'pwm_min': 60}


def start_fake(*extra):
    return subprocess.Popen([sys.executable, FAKE, '--link', PORT, '--log', LOG, '--boot-s', '0.3', *extra])


@pytest.mark.launch_test
def generate_test_description():
    bridge = launch_ros.actions.Node(package=PKG, executable='base_bridge',
                                     parameters=[PARAMS], output='screen')
    return launch.LaunchDescription([bridge, launch_testing.actions.ReadyToTest()]), {'bridge': bridge}


class TestBaseContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.path.exists(LOG):
            os.unlink(LOG)
        cls.fake = start_fake()
        rclpy.init()
        cls.node = rclpy.create_node('contract_tester')

    @classmethod
    def tearDownClass(cls):
        cls.fake.terminate(); cls.fake.wait()
        cls.node.destroy_node(); rclpy.shutdown()

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def fake_log(self):
        with open(LOG) as f:
            return [json.loads(line) for line in f]

    def test_tc022_imu_rate_and_shape(self):           # worked example
        msgs = []
        sub = self.node.create_subscription(Imu, 'imu/data_raw', msgs.append, qos_profile_sensor_data)
        self.spin_for(4.0)            # bridge reconnects (≤ 1 s) + fake boots (0.3 s)
        msgs.clear(); t0 = time.time()
        self.spin_for(2.0)
        rate = len(msgs) / (time.time() - t0)
        self.node.destroy_subscription(sub)
        self.assertGreaterEqual(rate, 18.0)
        m = msgs[-1]
        self.assertEqual(m.header.frame_id, 'imu_link')
        self.assertEqual(m.orientation_covariance[0], -1.0)
        a = m.linear_acceleration
        self.assertAlmostEqual((a.x**2 + a.y**2 + a.z**2) ** 0.5, 9.81, delta=0.3)

    # --- yours: TC-SCOUT-021a/b (cmd_vel -> M at 20 Hz, silence -> S < 0.3 s),
    #            TC-SCOUT-023 (pan/cmd 0.5 -> 'V 119' + joint_states),
    #            TC-SCOUT-024 (kill self.fake -> diagnostics ERROR < 1 s, start_fake() -> OK < 3 s),
    #            TC-SCOUT-024b (start_fake('--freeze-imu') -> imu_frozen true),
    #            /base/stop service.


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
```
**Check:** `colcon build && colcon test --packages-select scout_bringup --event-handlers console_direct+ && colcon test-result --verbose` → all TCs pass. Then run it **3 times**. A test that passes 2 out of 3 is a bug: find the race.

Commit: `feat(scout_base_py): base bridge node + SIL contract tests`. Open a PR for review.

## Questions (lab report)
1. Draw the threads in your node and every piece of shared state. Which lock or event protects each one?
2. What happens if `cmd_vel` arrives at 100 Hz? At 2 Hz? Is either a problem, and for whom?
3. Your service waits on an `Event`. What would happen in a single-threaded design, and why?
4. Why does the bridge send `S` on connect, before any command?
