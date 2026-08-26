cmake_minimum_required(VERSION 3.8)
project(uav_px4_backend)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Frame math lives here only; nodes call it in-process.
add_library(frame_conversions SHARED src/frame_conversions.cpp)
target_include_directories(frame_conversions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(frame_conversions
  Eigen3
  geometry_msgs
  px4_ros_com
)

install(TARGETS frame_conversions
  EXPORT export_frame_conversions
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

# Separate so that frame_conversions stays free of px4_msgs.
add_library(px4_interop SHARED src/px4_interop.cpp)
target_include_directories(px4_interop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(px4_interop
  rclcpp
  px4_msgs
  uav_interfaces
  sensor_msgs
)

install(TARGETS px4_interop
  EXPORT export_px4_interop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_frame_conversions HAS_LIBRARY_TARGET)
ament_export_targets(export_px4_interop HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(frame_conversions px4_interop)
ament_export_dependencies(
  Eigen3 geometry_msgs px4_ros_com rclcpp px4_msgs uav_interfaces sensor_msgs)

add_executable(px4_state_adapter_node src/px4_state_adapter_node.cpp)
target_link_libraries(px4_state_adapter_node frame_conversions px4_interop)
ament_target_dependencies(px4_state_adapter_node
  rclcpp
  px4_msgs
  uav_interfaces
  nav_msgs
  geometry_msgs
  sensor_msgs
)

add_executable(px4_frame_bridge_node src/px4_frame_bridge_node.cpp)
target_link_libraries(px4_frame_bridge_node frame_conversions px4_interop)
ament_target_dependencies(px4_frame_bridge_node
  rclcpp
  px4_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(px4_external_odometry_node src/px4_external_odometry_node.cpp)
target_link_libraries(px4_external_odometry_node frame_conversions px4_interop)
ament_target_dependencies(px4_external_odometry_node
  rclcpp
  px4_msgs
  uav_interfaces
  nav_msgs
  geometry_msgs
)

add_executable(px4_command_gateway_node src/px4_command_gateway_node.cpp)
target_link_libraries(px4_command_gateway_node frame_conversions px4_interop)
ament_target_dependencies(px4_command_gateway_node
  rclcpp
  px4_msgs
  uav_interfaces
  geometry_msgs
)

add_executable(offboard_session_manager_node src/offboard_session_manager_node.cpp)
target_link_libraries(offboard_session_manager_node px4_interop)
ament_target_dependencies(offboard_session_manager_node
  rclcpp
  px4_msgs
  uav_interfaces
)

install(TARGETS
  px4_state_adapter_node
  px4_frame_bridge_node
  px4_command_gateway_node
  offboard_session_manager_node
  px4_external_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Guard rail: a node test here could otherwise reach /fmu/in/*.
  ament_add_gtest(test_frame_conversions test/test_frame_conversions.cpp
    ENV ROS_DOMAIN_ID=94)
  target_link_libraries(test_frame_conversions frame_conversions)
  ament_target_dependencies(test_frame_conversions Eigen3 geometry_msgs px4_ros_com)

  ament_add_gtest(test_px4_interop test/test_px4_interop.cpp
    ENV ROS_DOMAIN_ID=94)
  target_link_libraries(test_px4_interop px4_interop)
  ament_target_dependencies(test_px4_interop rclcpp px4_msgs uav_interfaces sensor_msgs)
endif()

ament_package()
