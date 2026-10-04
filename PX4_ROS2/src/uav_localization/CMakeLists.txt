cmake_minimum_required(VERSION 3.8)
project(uav_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc video)

# No pose without its uncertainty.
add_library(source_channel SHARED src/source_channel.cpp)
target_include_directories(source_channel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(source_channel
  rclcpp
  uav_interfaces
  nav_msgs
  geometry_msgs
)

# Split out so tilt correction is testable standalone.
add_library(height_from_range SHARED src/height_from_range.cpp)
target_include_directories(height_from_range PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(height_from_range geometry_msgs)

# No ROS types, so the numbers can be pinned directly.
add_library(geodetic_projection SHARED src/geodetic_projection.cpp)
target_include_directories(geodetic_projection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Simulation-only fault injection; statistics testable without a running system.
add_library(gauss_markov_drift SHARED src/gauss_markov_drift.cpp)
target_include_directories(gauss_markov_drift PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(rangefinder_adapter_node src/rangefinder_adapter_node.cpp)
target_link_libraries(rangefinder_adapter_node height_from_range)
ament_target_dependencies(rangefinder_adapter_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# Makes a secretly-perfect source behave like a real one.
add_library(pose_degrader SHARED src/pose_degrader.cpp)
target_include_directories(pose_degrader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pose_degrader gauss_markov_drift)

# Safety-critical half of the mux: when switching is allowed.
add_library(source_selector SHARED src/source_selector.cpp)
target_include_directories(source_selector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# No OpenCV here, so the arithmetic stays pinned by tests.
add_library(optical_flow_scaling SHARED src/optical_flow_scaling.cpp)
target_include_directories(optical_flow_scaling PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(continuity_offset SHARED src/continuity_offset.cpp)
target_include_directories(continuity_offset PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(health_checks SHARED src/health_checks.cpp)
target_include_directories(health_checks PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(gps_adapter_node src/gps_adapter_node.cpp)
target_link_libraries(gps_adapter_node source_channel geodetic_projection gauss_markov_drift)
ament_target_dependencies(gps_adapter_node
  rclcpp
  uav_interfaces
  sensor_msgs
  nav_msgs
  geometry_msgs
)

add_executable(vio_adapter_node src/vio_adapter_node.cpp)
target_link_libraries(vio_adapter_node source_channel pose_degrader)
ament_target_dependencies(vio_adapter_node
  rclcpp
  uav_interfaces
  nav_msgs
  geometry_msgs
)

# A library so one test process can drive the real mux.
add_library(localization_mux_ros SHARED src/localization_mux_node.cpp)
target_include_directories(localization_mux_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(localization_mux_ros source_channel source_selector continuity_offset)
ament_target_dependencies(localization_mux_ros
  rclcpp
  uav_interfaces
  nav_msgs
  geometry_msgs
  std_msgs
)

add_executable(localization_mux_node src/localization_mux_main.cpp)
target_link_libraries(localization_mux_node localization_mux_ros)
ament_target_dependencies(localization_mux_node rclcpp)

add_executable(optical_flow_adapter_node src/optical_flow_adapter_node.cpp)
target_link_libraries(optical_flow_adapter_node optical_flow_scaling ${OpenCV_LIBS})
ament_target_dependencies(optical_flow_adapter_node
  rclcpp
  uav_interfaces
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
)

# A library so one test process can drive the real health node.
add_library(localization_health_ros SHARED src/localization_health_node.cpp)
target_include_directories(localization_health_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(localization_health_ros health_checks)
ament_target_dependencies(localization_health_ros
  rclcpp
  uav_interfaces
  nav_msgs
  geometry_msgs
  diagnostic_msgs
)

add_executable(localization_health_node src/localization_health_main.cpp)
target_link_libraries(localization_health_node localization_health_ros)
ament_target_dependencies(localization_health_node
  rclcpp
  uav_interfaces
  nav_msgs
  geometry_msgs
  diagnostic_msgs
)

install(TARGETS
  rangefinder_adapter_node
  gps_adapter_node
  vio_adapter_node
  localization_mux_node
  optical_flow_adapter_node
  localization_health_node
  DESTINATION lib/${PROJECT_NAME}
)

# Gate probe, not a unit test: it drives the real mux, so ctest must not own it.
install(PROGRAMS test/mux_fidelity_bench.py DESTINATION lib/${PROJECT_NAME})

install(TARGETS source_channel height_from_range geodetic_projection gauss_markov_drift
    pose_degrader source_selector optical_flow_scaling health_checks continuity_offset
    localization_mux_ros localization_health_ros
  EXPORT export_source_channel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_source_channel HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(
  source_channel height_from_range geodetic_projection gauss_markov_drift
  pose_degrader source_selector optical_flow_scaling health_checks continuity_offset
  localization_mux_ros localization_health_ros)
ament_export_dependencies(
  rclcpp uav_interfaces nav_msgs sensor_msgs geometry_msgs std_msgs diagnostic_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Keeps the channel's fabricated pose off a live domain.
  ament_add_gtest(test_source_channel test/test_source_channel.cpp
    ENV ROS_DOMAIN_ID=95)
  target_link_libraries(test_source_channel source_channel)
  ament_target_dependencies(test_source_channel rclcpp uav_interfaces nav_msgs geometry_msgs)

  ament_add_gtest(test_height_from_range test/test_height_from_range.cpp)
  target_link_libraries(test_height_from_range height_from_range)
  ament_target_dependencies(test_height_from_range geometry_msgs)

  ament_add_gtest(test_geodetic_projection test/test_geodetic_projection.cpp)
  target_link_libraries(test_geodetic_projection geodetic_projection)

  ament_add_gtest(test_gauss_markov_drift test/test_gauss_markov_drift.cpp)
  target_link_libraries(test_gauss_markov_drift gauss_markov_drift)

  ament_add_gtest(test_pose_degrader test/test_pose_degrader.cpp)
  target_link_libraries(test_pose_degrader pose_degrader)

  ament_add_gtest(test_source_selector test/test_source_selector.cpp)
  target_link_libraries(test_source_selector source_selector)
  ament_target_dependencies(test_source_selector uav_interfaces)

  ament_add_gtest(test_optical_flow_scaling test/test_optical_flow_scaling.cpp)
  target_link_libraries(test_optical_flow_scaling optical_flow_scaling)

  ament_add_gtest(test_health_checks test/test_health_checks.cpp)
  target_link_libraries(test_health_checks health_checks)

  ament_add_gtest(test_continuity_offset test/test_continuity_offset.cpp)
  target_link_libraries(test_continuity_offset continuity_offset)

  # Header-only and ROS-free: no domain to isolate.
  ament_add_gtest(test_twist_reading test/test_twist_reading.cpp)
  target_include_directories(test_twist_reading PRIVATE include)

  # This probe fabricates source odometry for the mux (R20).
  ament_add_gtest(test_localization_health_node test/test_localization_health_node.cpp
    TIMEOUT 120 ENV ROS_DOMAIN_ID=95)
  target_link_libraries(test_localization_health_node localization_health_ros)
  ament_target_dependencies(test_localization_health_node
    rclcpp uav_interfaces nav_msgs geometry_msgs diagnostic_msgs)

  ament_add_gtest(test_localization_mux_node test/test_localization_mux_node.cpp TIMEOUT 120
    ENV ROS_DOMAIN_ID=95)
  target_link_libraries(test_localization_mux_node localization_mux_ros)
  ament_target_dependencies(test_localization_mux_node
    rclcpp uav_interfaces nav_msgs geometry_msgs std_msgs)
endif()

ament_package()
