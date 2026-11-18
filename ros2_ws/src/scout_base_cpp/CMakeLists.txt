cmake_minimum_required(VERSION 3.8)
project(scout_base_cpp)

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED IMPORTED_TARGET libserial)

add_library(scout_base_core STATIC
  src/protocol.cpp
  src/kinematics.cpp
  src/conversions.cpp
)
target_include_directories(scout_base_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(base_bridge src/base_bridge_node.cpp)
target_link_libraries(base_bridge scout_base_core PkgConfig::LIBSERIAL)
ament_target_dependencies(base_bridge
  rclcpp geometry_msgs sensor_msgs std_msgs std_srvs diagnostic_msgs
)

install(TARGETS base_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_core test/test_core.cpp)
  target_link_libraries(test_core scout_base_core)
endif()

ament_package()
