cmake_minimum_required(VERSION 3.8)
project(uav_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uav_interfaces REQUIRED)

# Which goal may start in which state, and what makes a goal invalid. ROS-free so
# the whole matrix is pinned by tests: a wrong cell here flies the aircraft.
add_library(goal_admission SHARED src/goal_admission.cpp)
target_include_directories(goal_admission PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(goal_admission carrot)

# Setpoint stepping and yaw slewing. Kept ROS-free for the same reason: speed caps
# and the wrap at +-pi are arithmetic, and only a test catches them.
add_library(carrot SHARED src/carrot.cpp)
target_include_directories(carrot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Time-parameterized path. ROS-free: the limits are arithmetic, and only a test
# measuring the shipped positions can catch a violation at a segment join.
add_library(trajectory SHARED src/trajectory.cpp)
target_include_directories(trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(trajectory carrot)

# Rolling obstacle cost window. ROS-free: the inflation arithmetic and the
# stale-versus-empty distinction are only provable by a test that reads the cells.
add_library(costmap SHARED src/costmap.cpp)
target_include_directories(costmap PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(costmap carrot)

# Global route across the window. ROS-free: A* admissibility, the diagonal squeeze
# rule and the projection clock are arithmetic a test can pin exactly.
add_library(route_planner SHARED src/route_planner.cpp)
target_include_directories(route_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(route_planner costmap trajectory carrot)

# Reactive layer. ROS-free so the spiral, the vertical guard and the "255 is the
# window edge, not a wall" rule are pinned by arithmetic instead of by a flight.
add_library(local_avoidance SHARED src/local_avoidance.cpp)
target_include_directories(local_avoidance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(local_avoidance costmap carrot)

# Recovery geometry. ROS-free: refusing an unset home, refusing a descent and the
# altitude caps are arithmetic, and only a test can prove each one still bites.
add_library(recovery_planner SHARED src/recovery_planner.cpp)
target_include_directories(recovery_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(recovery_planner carrot)

# Split from the executable so one test process can drive the real action servers.
add_library(navigator_ros SHARED src/navigator_action_server_node.cpp)
target_include_directories(navigator_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(navigator_ros goal_admission carrot trajectory recovery_planner)
ament_target_dependencies(navigator_ros
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  uav_interfaces
)

add_library(route_planner_ros SHARED src/route_planner_node.cpp)
target_include_directories(route_planner_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(route_planner_ros route_planner costmap trajectory carrot)
ament_target_dependencies(route_planner_ros
  rclcpp
  nav_msgs
  geometry_msgs
  uav_interfaces
)

add_executable(route_planner_node src/route_planner_main.cpp)
target_link_libraries(route_planner_node route_planner_ros)

add_library(local_planner_ros SHARED src/local_planner_node.cpp)
target_include_directories(local_planner_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(local_planner_ros local_avoidance costmap carrot)
ament_target_dependencies(local_planner_ros
  rclcpp
  nav_msgs
  geometry_msgs
  uav_interfaces
)

add_executable(local_planner_node src/local_planner_main.cpp)
target_link_libraries(local_planner_node local_planner_ros)
ament_target_dependencies(route_planner_node rclcpp)

add_executable(navigator_action_server_node src/navigator_action_server_main.cpp)
target_link_libraries(navigator_action_server_node navigator_ros)
ament_target_dependencies(navigator_action_server_node rclcpp)

install(TARGETS navigator_action_server_node route_planner_node local_planner_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS goal_admission carrot trajectory costmap route_planner local_avoidance
    recovery_planner route_planner_ros local_planner_ros navigator_ros
  EXPORT export_goal_admission
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_goal_admission HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(goal_admission carrot trajectory costmap route_planner local_avoidance
  recovery_planner route_planner_ros local_planner_ros navigator_ros)
ament_export_dependencies(rclcpp rclcpp_action nav_msgs geometry_msgs uav_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_goal_admission test/test_goal_admission.cpp)
  target_link_libraries(test_goal_admission goal_admission carrot)

  ament_add_gtest(test_carrot test/test_carrot.cpp)
  target_link_libraries(test_carrot carrot)

  ament_add_gtest(test_trajectory test/test_trajectory.cpp TIMEOUT 120)
  target_link_libraries(test_trajectory trajectory carrot)

  ament_add_gtest(test_costmap test/test_costmap.cpp)
  target_link_libraries(test_costmap costmap carrot)

  ament_add_gtest(test_route_planner test/test_route_planner.cpp TIMEOUT 120)
  target_link_libraries(test_route_planner route_planner costmap trajectory carrot)

  ament_add_gtest(test_local_avoidance test/test_local_avoidance.cpp TIMEOUT 120)
  target_link_libraries(test_local_avoidance local_avoidance costmap carrot)

  # Links trajectory too: a recovery plan the ordinary core cannot fly is a bug.
  ament_add_gtest(test_recovery_planner test/test_recovery_planner.cpp TIMEOUT 120)
  target_link_libraries(test_recovery_planner recovery_planner trajectory carrot)

  # Same reason as below: this probe fabricates odometry_fused.
  ament_add_gtest(test_route_planner_node
    test/test_route_planner_node.cpp TIMEOUT 180
    ENV ROS_DOMAIN_ID=92)

  # Same reason: this probe fabricates odometry_fused (R20).
  ament_add_gtest(test_local_planner_node
    test/test_local_planner_node.cpp TIMEOUT 180
    ENV ROS_DOMAIN_ID=92)
  target_link_libraries(test_local_planner_node local_planner_ros local_avoidance costmap carrot)
  ament_target_dependencies(test_local_planner_node
    rclcpp nav_msgs geometry_msgs uav_interfaces)
  target_link_libraries(test_route_planner_node route_planner_ros route_planner costmap carrot)
  ament_target_dependencies(test_route_planner_node
    rclcpp nav_msgs geometry_msgs uav_interfaces)

  # Keeps the probe's fabricated vehicle state off a live domain.
  ament_add_gtest(test_navigator_action_server_node
    test/test_navigator_action_server_node.cpp TIMEOUT 900
    ENV ROS_DOMAIN_ID=92)
  target_link_libraries(test_navigator_action_server_node
    navigator_ros carrot trajectory recovery_planner)
  ament_target_dependencies(test_navigator_action_server_node
    rclcpp rclcpp_action nav_msgs geometry_msgs uav_interfaces)

  # G-N4a: the two real nodes (local_planner_node, navigator_action_server_node)
  # wired together, no Gazebo/PX4. Real-time flight at PX4's K_p 0.95, so this is
  # slower than the other node tests; the probe fabricates vehicle state (R20).
  ament_add_gtest(test_avoidance_chain
    test/test_avoidance_chain.cpp TIMEOUT 600
    ENV ROS_DOMAIN_ID=92)
  target_link_libraries(test_avoidance_chain
    navigator_ros carrot trajectory local_planner_ros local_avoidance costmap
    route_planner_ros route_planner)
  ament_target_dependencies(test_avoidance_chain
    rclcpp rclcpp_action nav_msgs geometry_msgs uav_interfaces)
endif()

ament_package()
