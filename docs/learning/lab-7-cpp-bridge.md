# Lab 7 — The same bridge in C++
**Objectives:** re-implement the base bridge in modern C++ (C++17) with rclcpp, with the **same** contract, and prove it by passing the **same** launch tests as the Python bridge. This is the C++ you'll talk about in interviews.
**Reading:** ROS 2 tutorials "Writing a simple publisher and subscriber (C++)", "Using parameters in a class (C++)", "Writing a simple service (C++)", "Testing: Writing basic tests with C++ with GTest"; LibSerial docs (https://libserial.readthedocs.io); cppreference on `std::optional`, `std::variant`, `std::thread`, `std::condition_variable`; Josh Newans, `diffdrive_arduino` (`include/diffdrive_arduino/arduino_comms.hpp`) for a LibSerial-based Arduino link.

## Background: what changes from Python
| Python (Lab 3–4) | C++ (this lab) |
|---|---|
| `dataclass` results, `None` for invalid | `struct ImuSample` …; `std::variant<ImuSample, Ack, WatchdogStop>` wrapped in `std::optional` |
| `float(x)` raising `ValueError` | `std::from_chars` (no exceptions, no locale) or `std::istringstream` with a fail check |
| `threading.Thread`, `Lock`, `Event` | `std::thread`, `std::mutex` + `std::lock_guard`, `std::condition_variable` + predicate |
| `pyserial` | `LibSerial::SerialPort` (`Open`, `SetBaudRate`, `ReadLine(line, '\n', timeout_ms)`, which throws `LibSerial::ReadTimeout`; `Write`) |
| GC, `finally:` | **RAII**: the destructor stops and joins the thread, sends `S`, closes the port |
| `pytest` | `gtest` via `ament_add_gtest` |

---

## Task 7.1 — Package and build 🧠
```bash
sudo apt install libserial-dev           # 1.0.0 on Ubuntu 24.04
ros2 pkg create scout_base_cpp --build-type ament_cmake --license Apache-2.0 \
  --maintainer-name "Jaime Tellie" --maintainer-email jaime.tellie@gmail.com \
  --dependencies rclcpp geometry_msgs sensor_msgs std_msgs std_srvs diagnostic_msgs
```
- `package.xml`: `<depend>libserial-dev</depend>`, `<test_depend>ament_cmake_gtest</test_depend>`.
- Layout: `include/scout_base_cpp/{protocol,kinematics,conversions}.hpp`, `src/{protocol,kinematics,conversions}.cpp`, `src/base_bridge_node.cpp`, `test/test_core.cpp`.
- `CMakeLists.txt`, the parts that matter:
  ```cmake
  set(CMAKE_CXX_STANDARD 17)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBSERIAL REQUIRED libserial)

  add_library(scout_base_core src/protocol.cpp src/kinematics.cpp src/conversions.cpp)
  target_include_directories(scout_base_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

  add_executable(base_bridge src/base_bridge_node.cpp)
  target_link_libraries(base_bridge scout_base_core ${LIBSERIAL_LIBRARIES})
  target_include_directories(base_bridge PRIVATE ${LIBSERIAL_INCLUDE_DIRS})
  ament_target_dependencies(base_bridge rclcpp geometry_msgs sensor_msgs std_msgs std_srvs diagnostic_msgs)
  install(TARGETS base_bridge DESTINATION lib/${PROJECT_NAME})

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_core test/test_core.cpp)
    target_link_libraries(test_core scout_base_core)
  endif()
  ```
  The core library has **no** ROS dependency, the same split as in Python.
- Build with compile commands so VS Code IntelliSense works:
  `colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

## Task 7.2 — Port the unit tests first 🧠
Translate U1–U8 from Lab 3 into `test/test_core.cpp`: **the same cases, the same numbers**. Use `TEST()` and parameterised tests (`TEST_P`) for the list of bad lines. RED first.
**Check:** `colcon test --packages-select scout_base_cpp` builds and runs them, and they fail.

## Task 7.3 — Core in C++ 🧠
- `std::optional<Line> parse_line(std::string_view)`, where `using Line = std::variant<ImuSample, Ack, WatchdogStop>;`
- `std::pair<int,int> twist_to_pwm(double v, double w, const DriveParams&)`
- `StaleDetector`, `RateMeter` (take `double now`), `link_level(...)`

**Check:** all gtests green; `ament_cpplint`/`uncrustify` clean if you enable `ament_lint_auto`.

## Task 7.4 — The node 🧠
`class BaseBridge : public rclcpp::Node` in `base_bridge_node.cpp`, node name `base_bridge`:
- parameters: `declare_parameter<double>("track_width", 0.0)` …, the same names as IF-ROS-TOPICS §8
- QoS: `rclcpp::SensorDataQoS()` for `imu/data_raw` and `base/debug/yaw`; `rclcpp::QoS(10)` for the rest
- timers: `create_wall_timer(std::chrono::milliseconds(50), ...)`
- reader: `std::thread reader_` plus `std::atomic<bool> running_`. The loop calls `port_.ReadLine(line, '\n', 100)` in a `try`, where `ReadTimeout` → continue, and any other exception → close + reconnect.
- writes under `std::mutex write_mutex_`. Acks via `std::condition_variable ack_cv_` + `std::mutex ack_mutex_` + `wait_for(lock, timeout, predicate)`.
- **destructor:** `running_ = false;` → `reader_.join();` → write `S` → `port_.Close()`. Never let a joinable `std::thread` be destroyed (that calls `std::terminate`).
- `main`: `rclcpp::init`; `rclcpp::spin(std::make_shared<BaseBridge>())`; `rclcpp::shutdown()`.

**Check:** the Lab 4 manual checks against the fake R4 give the same results as the Python bridge.

## Task 7.5 — One contract, two implementations 🧠
```bash
SCOUT_BRIDGE_IMPL=cpp colcon test --packages-select scout_bringup --event-handlers console_direct+
ros2 launch scout_bringup base.launch.py impl:=cpp foxglove:=true
```
**Check:** all TC-SCOUT-021…024 pass for `cpp` as they did for `py` (REQ-SCOUT-024). Compare the two:
- CPU in `top` (`%CPU` of each bridge)
- receive-to-publish latency: log `now − stamp` around `publish`, or add a temporary debug topic

Put both in the lab report.

## Questions (lab report)
1. What happens if the `BaseBridge` destructor runs while the reader thread is blocked in `ReadLine` with no timeout? How does your design avoid that?
2. Why `std::variant` instead of a base class `Line` with virtual methods? When would you choose inheritance?
3. Is `std::from_chars` for `double` safe across locales? Why does that matter on a robot whose OS might be set to Dutch (`0,5` vs `0.5`)?
4. Python vs C++ numbers: which differences matter at 20 Hz, and which would at 1 kHz (ros2_control)?
