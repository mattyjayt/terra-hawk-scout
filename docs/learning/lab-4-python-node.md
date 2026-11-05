# Lab 4 — Python bridge node + software-in-the-loop tests
**Status (4 Nov):** Tasks 4.1–4.3 done; 4.0 and 4.4–4.6 rewritten step by step.
**Objectives:** wrap the Lab 3 core in a ROS 2 node that implements IF-ROS-TOPICS P1 completely, and prove it against the fake R4 with an automated launch test.
**Reading:** ROS 2 tutorials "Writing a simple publisher and subscriber (Python)", "Using parameters in a class (Python)", "Writing a simple service and client (Python)"; Concepts "About QoS settings" and "Executors"; DD-SCOUT-BRIDGE §3–§7; the launch_testing README (https://github.com/ros2/launch/tree/jazzy/launch_testing).

## Background
- **Executor and callbacks:** `rclpy.spin(node)` runs your subscription, timer and service callbacks one at a time (single-threaded executor). A callback that blocks stops every other callback. That's why the serial *reader* is a separate thread (DD-SCOUT-BRIDGE §3).
- **Threads in Python:** the reader thread and the callbacks share state, such as the last ack, the link state and the serial port for writing. Protect writes with a `threading.Lock`, and signal acks with a `threading.Event`. The GIL protects the interpreter, not your logic.
- **QoS:** a best-effort subscriber can read a reliable publisher, but not the reverse. Use the profiles from IF-ROS-TOPICS §3.

---

## How this lab is written
Each task lists the **state** to add in `__init__`, every **method** with its exact signature, the **steps** inside each method, and a **test** with pass criteria.
- Write the methods as empty stubs first, so the node starts. Then fill in one method at a time and test after each one.
- Build once with `colcon build --symlink-install`. After that, Python edits take effect without rebuilding.
- Use absolute paths in commands. Paths below assume the repo is at `~/Projects/terra-hawk-scout`.

**Three-terminal test setup** (used by every task):
```bash
# T1 — fake R4 (logs every line it receives)
python3 ~/Projects/terra-hawk-scout/tools/fake_r4.py --log /tmp/r4.jsonl
# T2 — your bridge
ros2 run scout_base_py base_bridge --ros-args \
  --params-file ~/Projects/terra-hawk-scout/ros2_ws/src/scout_bringup/config/base.yaml -p port:=/tmp/fake_r4
# T3 — what the R4 received
tail -f /tmp/r4.jsonl
```

---

## Task 4.0: Lab 3 fixes first (do this before 4.4) 🧠
Tasks 4.4–4.6 call these functions, and their tests will fail until they're fixed. The full list is in PR #19.
| Fix | Caught again by |
|---|---|
| `servo_deg_to_pan_rad`: subtract 90 first (90° → 0 rad) | 4.4 (joint state at centre must be 0.0) |
| `pan_rad_to_servo_deg`: convert only once; clamp **radians** to `pan_min/pan_max`, then convert (0 rad → 90, 0.5 rad → 119) | 4.5 |
| `StaleDetector`: compare with the **previous** tuple, not with zeros | 4.6 (`--freeze-imu`) |
| `RateMeter`: a true sliding window (keep timestamps, drop the old ones) | 4.6 (rate must stay ~20) |
| `parse_line`: `line = line.strip()` first (the R4 sends `\r\n`) | 4.6 (watchdog stops counted) |
| rename `gravity_x/y/z` → `gyro_x/y/z` | 4.4 |
| `scout_bringup`: add `launch/.gitkeep` and `foxglove/.gitkeep`; fix the pep257 docstrings | CI (PR #20) |
Add a unit test for each fix (RED first), then commit: `fix(scout_base_py): Lab 3 review fixes`.

---

## Task 4.1: Node skeleton ✅ (done 3–4 Nov)
Parameters from IF-ROS-TOPICS §8 (the defaults must match that table), five publishers, two subscriptions (relative names: `cmd_vel`, `pan/cmd`), and the refuse-to-start check for `track_width`, `max_wheel_speed` and `pwm_min`.

## Task 4.2: Serial link and reader thread ✅ (done 4 Nov)
Methods `_open_port() -> bool`, `_close_port() -> None`, `_write_line(text) -> bool`, `_reader_loop() -> None`.
Verified: waits once for the port, sends `S` on connect, reads 20 I lines/s, logs "R4 link lost" and reconnects.

## Task 4.3: Drive with timeout ✅ (done 4 Nov)
`cmd_vel_callback` stores `(v, w)` plus `time.monotonic()`. `_send_tick()` sends `M l r` at 20 Hz while the command is fresh, else one `S`.
Verified: `M 70 70` at 20.0 Hz, a single `S` after the timeout, and a left turn gives `M -70 70`.

---

## Task 4.4: Telemetry, turning `I` lines into ROS messages 🧠
**Goal:** every `I` line becomes an `Imu` and a `JointState` message plus a debug yaw; the battery voltage is averaged and published at 1 Hz. Field-by-field rules: IF-ROS-TOPICS §5.2 (T3, T4, T5, T7).

**New imports**
```python
import collections
import math
from scout_base_py.conversions import dps_to_radps, g_to_mps2, RateMeter, StaleDetector
from scout_base_py.kinematics import servo_deg_to_pan_rad
from scout_base_py.protocol import Ack, WatchdogStop    # used in 4.5 / 4.6
```

**State in `__init__`** (read parameters once):
```python
self._frame_imu = self.get_parameter('frame_id_imu').value
self._frame_base = self.get_parameter('frame_id_base').value
self._gyro_var = self.get_parameter('gyro_var').value
self._accel_var = self.get_parameter('accel_var').value
self._pan_sign = self.get_parameter('pan_sign').value
self._cell_count = self.get_parameter('cell_count').value
self._battery_present_v = self.get_parameter('battery_present_v').value
self._batt_samples = collections.deque(maxlen=20)       # last 20 voltages = 1 s
self._stale = StaleDetector(self.get_parameter('imu_stale_count').value)
self._imu_frozen = False
self._imu_rate = RateMeter(1.0)
self._last_sample_time = None                            # monotonic time of last I line
self._battery_timer = self.create_timer(1.0, self._publish_battery)
```

**Method 1: `_handle_line(self, msg) -> None`** (the dispatcher, called by the reader thread)
1. `ImuSample` → `self._on_imu(msg)`
2. `Ack` → `self._on_ack(msg)` (an empty stub for now; written in 4.5)
3. `WatchdogStop` → `self._on_watchdog()` (an empty stub for now; written in 4.6)
4. anything else (`None`) → ignore for now (4.6 counts it)

In `_reader_loop`, replace the `i_count` block with `self._handle_line(parse_line(raw.decode(errors='replace')))`. The new `RateMeter` replaces your per-second "I lines" log.

**Method 2: `_on_imu(self, s: ImuSample) -> None`**
1. `stamp = self.get_clock().now().to_msg()`. All three messages use the same stamp.
2. **Imu** (`m = Imu()`):
   - `m.header.stamp = stamp`; `m.header.frame_id = self._frame_imu`
   - `m.orientation.w = 1.0`; `m.orientation_covariance[0] = -1.0` (REP-145: "no orientation")
   - `m.angular_velocity.x/y/z = dps_to_radps(s.gyro_x / _y / _z)`
   - `m.linear_acceleration.x/y/z = g_to_mps2(s.accel_x / _y / _z)`
   - covariances: a 9-element list, the diagonal at indices 0, 4 and 8:
     `m.angular_velocity_covariance = [g, 0.0, 0.0, 0.0, g, 0.0, 0.0, 0.0, g]` with `g = self._gyro_var` (the same pattern with `self._accel_var`)
   - `self.imu_publisher.publish(m)`
3. **JointState**: `header.stamp = stamp`, `name = ['pan_joint']`, `position = [servo_deg_to_pan_rad(s.servo_deg, self._pan_sign)]`, then publish.
4. **Debug yaw**: `Float64(data=math.radians(s.yaw_deg))`, then publish on `base_debug_yaw_publisher`.
5. **Bookkeeping**:
   - `self._batt_samples.append(s.batt_voltage)`
   - `self._imu_frozen = self._stale.update((s.accel_x, s.accel_y, s.accel_z, s.gyro_x, s.gyro_y, s.gyro_z))`
   - `now = time.monotonic()`; `self._imu_rate.tick(now)`; `self._last_sample_time = now`

Publishing from the reader thread is allowed in rclpy.

**Method 3: `_publish_battery(self) -> None`** (1 Hz timer)
1. If `self._batt_samples` is empty: return.
2. `v = sum(...) / len(...)`
3. `b = BatteryState()`, then set:
   - `header.stamp` (now) and `header.frame_id = self._frame_base`
   - `voltage = v`; `present = v > self._battery_present_v`
   - `power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION`; status and health stay `UNKNOWN` (0)
   - `current`, `charge`, `capacity`, `design_capacity`, `percentage`, `temperature` = `float('nan')`
   - `cell_voltage = [v / self._cell_count] * self._cell_count`; `location = 'base'`
4. Publish.

**Test 4.4** (setup above, plus a T4):
```bash
ros2 topic hz /imu/data_raw                         # ≈ 20
ros2 topic echo --once /imu/data_raw                # frame_id imu_link, orientation_covariance[0] -1.0,
                                                    # linear_acceleration.z ≈ 9.81 (fake R4: 1 g on z)
ros2 topic echo --once /joint_states                # name [pan_joint], position [0.0]  (1.571 = Lab 3 bug)
ros2 topic echo --once /battery                     # voltage ≈ 7.6, present true, percentage .nan
ros2 topic echo --once /base/debug/yaw
```
Commit: `feat(scout_base_py): telemetry publishers (Lab 4.4)`.

---

## Task 4.5: Pan command and services 🧠
**Goal:** `pan/cmd` moves the servo; three `std_srvs/Trigger` services send `S`, `C` and `Z` and **wait for the R4's acknowledgement**. Contract: IF-ROS-TOPICS T2 and §6.

**Background: why waiting works here.** A service callback runs in the ROS executor. The `A … ok` reply arrives in your **reader thread**. A `threading.Event` lets one thread sleep until another one wakes it: the service calls `event.wait(timeout)` and the reader calls `event.set()`. That only works because reading happens in a *different* thread. If you read the port inside the service callback instead, nothing reads while it waits, and it hangs. Try that once, to see it.

**New imports:** `from std_srvs.srv import Trigger` and `from scout_base_py.kinematics import pan_rad_to_servo_deg`; `format_servo` from protocol.

**State in `__init__`**
```python
self._pan_min = self.get_parameter('pan_min').value
self._pan_max = self.get_parameter('pan_max').value
self._last_servo_deg = None
self._last_servo_time = 0.0
self._ack_event = threading.Event()
self._last_ack = ''
self.create_service(Trigger, 'base/stop', self._srv_stop)
self.create_service(Trigger, 'imu/calibrate', self._srv_calibrate)
self.create_service(Trigger, 'imu/zero_yaw', self._srv_zero_yaw)
```

**Method 1: `pan_cmd_callback(self, msg) -> None`** (replace the logging version)
1. `deg = pan_rad_to_servo_deg(msg.data, self._pan_sign, self._pan_min, self._pan_max)`
2. If `self._last_servo_deg is not None` and `abs(deg - self._last_servo_deg) < 1`: return (no change).
3. If `time.monotonic() - self._last_servo_time < 0.05`: return (max 20 Hz; hobby servos jitter when flooded).
4. If `self._write_line(format_servo(deg))`: store `deg` and the time.

**Method 2: `_on_ack(self, ack: Ack) -> None`** (called by `_handle_line`)
`self._last_ack = ack.text`, then `self._ack_event.set()`.

**Method 3: `_send_and_wait(self, line: str, expect: str, timeout: float) -> tuple[bool, str]`**
1. If `self._ser is None`: return `(False, 'link down')`.
2. `self._ack_event.clear()`
3. If not `self._write_line(line)`: return `(False, 'write failed')`.
4. `deadline = time.monotonic() + timeout`. Loop while time remains:
   - `if not self._ack_event.wait(deadline - time.monotonic()): break` (timed out)
   - `if self._last_ack == expect: return (True, expect)`
   - otherwise `self._ack_event.clear()` and keep waiting. Other acks arrive first, e.g. `C calibrating 2 s` before `C ok`.
5. Return `(False, f'no "{expect}" within {timeout} s')`.

**Methods 4–6: the service callbacks.** The signature is always `(self, request, response)`, and they **return** `response`:
| Method | Calls | Extra |
|---|---|---|
| `_srv_stop` | `_send_and_wait('S\n', 'S ok', 0.5)` | first set `self._cmd_time = None`, so `_send_tick` doesn't start driving again |
| `_srv_calibrate` | `_send_and_wait('C\n', 'C ok', 3.0)` | the motors stop and the car must stay still for 2 s |
| `_srv_zero_yaw` | `_send_and_wait('Z\n', 'Z ok', 0.5)` | |
In each: `ok, text = ...`, then `response.success = ok`, `response.message = text`, `return response`.

**Test 4.5**
```bash
ros2 topic pub --once /pan/cmd std_msgs/msg/Float64 "{data: 0.5}"     # T3 shows V 119
ros2 topic pub --once /pan/cmd std_msgs/msg/Float64 "{data: 0.0}"     # V 90
ros2 service call /base/stop std_srvs/srv/Trigger                     # success=True, message='S ok'
                                                                      # (False, 'no "S ok"…'? → the \r\n fix in Task 4.0)
ros2 service call /imu/calibrate std_srvs/srv/Trigger                 # takes ~2 s, success=True
ros2 service call /imu/zero_yaw std_srvs/srv/Trigger                  # success=True
# stop T1 (fake R4), then:
ros2 service call /base/stop std_srvs/srv/Trigger                     # success=False, message='link down'
```
Commit: `feat(scout_base_py): pan command and services (Lab 4.5)`.

---

## Task 4.6: Diagnostics and clean shutdown 🧠
**Goal:** one `DiagnosticStatus` per second reporting the link's health (IF-ROS-TOPICS T6), and on Ctrl+C the R4 always receives a final `S`.

**New imports:** `from diagnostic_msgs.msg import DiagnosticStatus, KeyValue`; `from scout_base_py.conversions import link_level`; `from rclpy.executors import ExternalShutdownException`.

**State in `__init__`**
```python
self._port = self.get_parameter('port').value
self._lines_rx = 0
self._parse_errors = 0
self._watchdog_stops = 0
self._diag_timer = self.create_timer(1.0, self._publish_diagnostics)
```

**Small additions**
- In `_handle_line`: `self._lines_rx += 1` for every line. If `msg is None`, `self._parse_errors += 1`. The reader loop already skips empty reads, so `None` here means a garbled or unknown line.
- `_on_watchdog(self) -> None`: `self._watchdog_stops += 1`.
- `_on_ack`: also store the text for the diagnostics (`last_ack`).

**Method 1: `_publish_diagnostics(self) -> None`** (1 Hz timer)
1. `now = time.monotonic()`; `rate = self._imu_rate.rate(now)`
2. `since = now - self._last_sample_time if self._last_sample_time is not None else float('inf')`
3. `level = link_level(self._ser is not None, rate, 20.0, self._imu_frozen, since)` (20.0 = the serial-v1 telemetry rate)
4. `message`: pick a short text per level, e.g. `'ok'`, `'low rate'`, `'link down'` / `'no data'` / `'IMU frozen'`.
5. `st = DiagnosticStatus()`, then set:
   - `st.level = bytes([level])` (the message field is a `bytes`; `DiagnosticStatus.OK/WARN/ERROR` are `b'\x00'/b'\x01'/b'\x02'`)
   - `st.name = 'base_bridge: R4 link'`; `st.message = message`; `st.hardware_id = self._port`
   - `st.values = [KeyValue(key='port', value=self._port), KeyValue(key='connected', value=str(self._ser is not None)), ...]` with **all values as strings**: `line_rate_hz` (e.g. `f'{rate:.1f}'`), `lines_rx`, `parse_errors`, `watchdog_stops`, `imu_frozen`, `last_ack`, `protocol` = `'serial-r4-v1'`
6. `arr = DiagnosticArray()`; `arr.header.stamp = self.get_clock().now().to_msg()`; `arr.status = [st]`; publish.

**Method 2: `close(self) -> None`** (called from `main`)
1. `self._write_line('S\n')`: stop the motors **first**
2. `self._running = False`
3. `self._reader.join(timeout=1.0)`: wait for the thread to finish its current `readline()` (≤ 0.1 s)
4. `self._close_port()`

**`main()`**
```python
def main(args=None):
    rclpy.init(args=args)
    node = BaseBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
```

**Test 4.6**
```bash
ros2 topic echo /diagnostics          # level b'\x00', message 'ok', line_rate_hz ≈ 20.0
# stop T1           → within 1 s: level b'\x02', connected 'False'
# start T1 again    → within 3 s: back to b'\x00'
# restart T1 with:  python3 ~/Projects/terra-hawk-scout/tools/fake_r4.py --log /tmp/r4.jsonl --freeze-imu
#                   → imu_frozen 'True', level b'\x02'   (stays OK? → StaleDetector, Task 4.0)
# Ctrl+C on T2      → the last line in T3 is {"rx": "S"}
```
Commit: `feat(scout_base_py): diagnostics and clean shutdown (Lab 4.6)`.

---

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
