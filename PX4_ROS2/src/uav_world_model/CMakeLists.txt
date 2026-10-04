cmake_minimum_required(VERSION 3.8)
project(uav_world_model)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(uav_perception REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Free of ROS types, so the numbers can be pinned directly.
add_library(world_model_core SHARED
  src/frame_transforms.cpp
  src/pose_buffer.cpp
  src/uncertainty.cpp
  src/world_map.cpp
)
target_include_directories(world_model_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# One owner for the optical/body convention; divergence would be silent.
ament_target_dependencies(world_model_core uav_perception)

# Split from the executable so one test process drives the real node.
add_library(world_model_ros SHARED src/world_model_node.cpp)
target_include_directories(world_model_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(world_model_ros world_model_core)
ament_target_dependencies(world_model_ros
  rclcpp
  uav_interfaces
  nav_msgs
  geometry_msgs
)

add_executable(world_model_node src/world_model_main.cpp)
target_link_libraries(world_model_node world_model_ros)
ament_target_dependencies(world_model_node rclcpp)

install(TARGETS world_model_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS world_model_core world_model_ros
  EXPORT export_world_model_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_world_model_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(world_model_core world_model_ros)
ament_export_dependencies(rclcpp uav_interfaces uav_perception nav_msgs geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frame_transforms test/test_frame_transforms.cpp)
  target_link_libraries(test_frame_transforms world_model_core)

  ament_add_gtest(test_uncertainty test/test_uncertainty.cpp)
  target_link_libraries(test_uncertainty world_model_core)

  ament_add_gtest(test_pose_buffer test/test_pose_buffer.cpp)
  target_link_libraries(test_pose_buffer world_model_core)

  ament_add_gtest(test_world_map test/test_world_map.cpp)
  target_link_libraries(test_world_map world_model_core)

  # Keeps the probe's fabricated fused pose off a live domain.
  ament_add_gtest(test_world_model_node test/test_world_model_node.cpp TIMEOUT 120
    ENV ROS_DOMAIN_ID=91)
  target_link_libraries(test_world_model_node world_model_ros)
  # Pins the shipped mount numbers, not a copy of them.
  target_compile_definitions(test_world_model_node PRIVATE
    UAV_WORLD_MODEL_SHIPPED_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/world_model_params.yaml")
  ament_target_dependencies(test_world_model_node
    rclcpp uav_interfaces nav_msgs geometry_msgs)
endif()

ament_package()
