cmake_minimum_required(VERSION 3.8)
project(scout_localization)

# C++17: std::optional
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ---- core: pure C++, no ROS (unit-tested) ----
add_library(scout_localization_core STATIC
  src/scan_math.cpp
  src/odom_math.cpp
  src/safety.cpp
)

target_include_directories(scout_localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---- nodes: thin ROS shells around the core ----
add_executable(scan_probe src/scan_probe_node.cpp)
target_link_libraries(scan_probe scout_localization_core)
ament_target_dependencies(scan_probe rclcpp sensor_msgs)
install(TARGETS scan_probe DESTINATION lib/${PROJECT_NAME})

add_executable(covariance_relay src/covariance_relay_node.cpp)
target_link_libraries(covariance_relay scout_localization_core)
ament_target_dependencies(covariance_relay rclcpp nav_msgs)
install(TARGETS covariance_relay DESTINATION lib/${PROJECT_NAME})

add_executable(safety_stop src/safety_stop_node.cpp)
target_link_libraries(safety_stop scout_localization_core)
ament_target_dependencies(safety_stop rclcpp sensor_msgs geometry_msgs)
install(TARGETS safety_stop DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_core
    test/test_scan_math.cpp
    test/test_odom_math.cpp
    test/test_safety.cpp
  )
  target_link_libraries(test_core scout_localization_core)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_safety_stop_contract.py TIMEOUT 90)
endif()

ament_package()
