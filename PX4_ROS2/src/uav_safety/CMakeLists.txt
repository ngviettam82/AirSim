cmake_minimum_required(VERSION 3.8)
project(uav_safety)

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
find_package(nav_msgs REQUIRED)

# Policy/FSM arithmetic (S:3 detection table + S:4.3 FSM + S:3.1 ClearFault).
# ROS-free, same rule as uav_control_authority's authority_arbiter: a
# safety-critical matrix is pinned by a test, not by a flight. P8.3 links a
# node executable against this target; nothing here pulls rclcpp/uav_interfaces.
add_library(failsafe_policy SHARED src/failsafe_policy.cpp)
target_include_directories(failsafe_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Split from the executable so one test process can drive the real node
# (same shape as uav_control_authority's control_authority_ros).
add_library(safety_supervisor_ros SHARED src/safety_supervisor_node.cpp)
target_include_directories(safety_supervisor_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(safety_supervisor_ros failsafe_policy)
ament_target_dependencies(safety_supervisor_ros
  rclcpp
  uav_interfaces
  diagnostic_msgs
  nav_msgs
)

add_executable(safety_supervisor_node src/safety_supervisor_main.cpp)
target_link_libraries(safety_supervisor_node safety_supervisor_ros)
ament_target_dependencies(safety_supervisor_node rclcpp)

install(TARGETS safety_supervisor_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS failsafe_policy safety_supervisor_ros
  EXPORT export_failsafe_policy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_failsafe_policy HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(failsafe_policy safety_supervisor_ros)
ament_export_dependencies(rclcpp uav_interfaces diagnostic_msgs nav_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # No ROS_DOMAIN_ID env needed (R20): no rclcpp::init, no domain participant.
  ament_add_gtest(test_failsafe_policy test/test_failsafe_policy.cpp)
  target_link_libraries(test_failsafe_policy failsafe_policy)

  # This probe fabricates traffic on 12 real topics (R20): isolated domain.
  ament_add_gtest(test_safety_supervisor_node
    test/test_safety_supervisor_node.cpp TIMEOUT 180
    ENV ROS_DOMAIN_ID=97)
  target_link_libraries(test_safety_supervisor_node
    safety_supervisor_ros failsafe_policy)
  ament_target_dependencies(test_safety_supervisor_node
    rclcpp uav_interfaces diagnostic_msgs nav_msgs)

  # G-S1: the REAL control_authority_manager_node (control_authority_ros)
  # runs in the SAME process as the real safety node -- the cut is proven
  # through the actual arbiter, not a stand-in. Isolated domain (R20): both
  # nodes fabricate/consume traffic on real topics.
  find_package(uav_control_authority REQUIRED)
  ament_add_gtest(test_safety_cut_chain
    test/test_safety_cut_chain.cpp TIMEOUT 300
    ENV ROS_DOMAIN_ID=97)
  target_link_libraries(test_safety_cut_chain
    safety_supervisor_ros failsafe_policy)
  ament_target_dependencies(test_safety_cut_chain
    rclcpp uav_interfaces diagnostic_msgs nav_msgs uav_control_authority)
endif()

ament_package()
