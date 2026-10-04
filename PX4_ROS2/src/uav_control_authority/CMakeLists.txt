cmake_minimum_required(VERSION 3.8)
project(uav_control_authority)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Priority + hysteresis + latch arithmetic. ROS-free so the safety-critical
# matrix is pinned by a test, not by a flight (same rule as uav_navigation's
# carrot/costmap/goal_admission).
add_library(authority_arbiter SHARED src/authority_arbiter.cpp)
target_include_directories(authority_arbiter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Split from the executable so one test process can drive the real node.
add_library(control_authority_ros SHARED src/control_authority_manager_node.cpp)
target_include_directories(control_authority_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(control_authority_ros authority_arbiter)
ament_target_dependencies(control_authority_ros
  rclcpp
  uav_interfaces
  diagnostic_msgs
)

add_executable(control_authority_manager_node src/control_authority_manager_main.cpp)
target_link_libraries(control_authority_manager_node control_authority_ros)
ament_target_dependencies(control_authority_manager_node rclcpp)

install(TARGETS control_authority_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS authority_arbiter control_authority_ros
  EXPORT export_authority_arbiter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_authority_arbiter HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(authority_arbiter control_authority_ros)
ament_export_dependencies(rclcpp uav_interfaces diagnostic_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_authority_arbiter test/test_authority_arbiter.cpp)
  target_link_libraries(test_authority_arbiter authority_arbiter)

  # This probe fabricates ControlCommand on all 4 source topics (R20).
  ament_add_gtest(test_control_authority_manager_node
    test/test_control_authority_manager_node.cpp TIMEOUT 180
    ENV ROS_DOMAIN_ID=96)
  target_link_libraries(test_control_authority_manager_node
    control_authority_ros authority_arbiter)
  ament_target_dependencies(test_control_authority_manager_node
    rclcpp uav_interfaces diagnostic_msgs)
endif()

ament_package()
