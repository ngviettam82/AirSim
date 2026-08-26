cmake_minimum_required(VERSION 3.8)
project(uav_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

# P9.2: reactive mission guard table (MissionWorldView -> GuardVerdict).
# ROS-free -- same rule as uav_safety's failsafe_policy: a safety-critical
# decision table is pinned by a test, not by a flight. mission_executor_node
# (P9.4) will link against this; nothing here pulls rclcpp/uav_interfaces/BT.
add_library(mission_policy SHARED src/mission_policy.cpp)
target_include_directories(mission_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# P9.3: mission catalog + mission_params parsing/validation. ROS-free; only
# a YAML/XML parser -- P9.4 owns actual BT tree construction, not this lib.
add_library(mission_registry SHARED src/mission_registry.cpp)
target_include_directories(mission_registry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mission_registry
  yaml-cpp
  tinyxml2::tinyxml2
)

# P9.4: at-most-one-goal-navigator-forever FSM. ROS-free (nav_goal_broker.hpp
# includes no rclcpp_action) -- mission_executor_ros wires it to the 7 real
# navigator action clients.
add_library(nav_goal_broker SHARED src/nav_goal_broker.cpp)
target_include_directories(nav_goal_broker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# P9.4: BT engine + NavGoalBroker + action server + 4 services. Split from
# the executable so one test process can drive the real node (same shape as
# uav_safety's safety_supervisor_ros / uav_control_authority's
# control_authority_ros).
add_library(mission_executor_ros SHARED
  src/bt_nodes.cpp
  src/mission_executor_node.cpp
)
target_include_directories(mission_executor_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mission_executor_ros
  mission_policy
  mission_registry
  nav_goal_broker
  # BT::behaviortree_cpp_v3, not behaviortree_cpp_v3::behaviortree_cpp_v3:
  # this apt install's lib/cmake/ raw-CMake export shadows the ament one in
  # share/cmake/ in find_package() search order (both point at the same
  # .so; verified via --debug-find).
  BT::behaviortree_cpp_v3
  ament_index_cpp::ament_index_cpp
)
ament_target_dependencies(mission_executor_ros
  rclcpp
  rclcpp_action
  uav_interfaces
  nav_msgs
  diagnostic_msgs
)

add_executable(mission_executor_node src/mission_executor_main.cpp)
target_link_libraries(mission_executor_node mission_executor_ros)
ament_target_dependencies(mission_executor_node rclcpp)

install(TARGETS mission_executor_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS mission_policy mission_registry nav_goal_broker mission_executor_ros
  EXPORT export_mission_policy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_mission_policy HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(mission_policy mission_registry nav_goal_broker mission_executor_ros)
ament_export_dependencies(
  yaml_cpp_vendor tinyxml2_vendor rclcpp rclcpp_action uav_interfaces nav_msgs
  diagnostic_msgs behaviortree_cpp_v3 ament_index_cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(uav_control_authority REQUIRED)

  # No ROS_DOMAIN_ID env needed (R20): no rclcpp::init, no domain participant.
  ament_add_gtest(test_mission_policy test/test_mission_policy.cpp)
  target_link_libraries(test_mission_policy mission_policy)

  ament_add_gtest(test_nav_goal_broker test/test_nav_goal_broker.cpp)
  target_link_libraries(test_nav_goal_broker nav_goal_broker)

  ament_add_gtest(test_mission_registry test/test_mission_registry.cpp)
  target_link_libraries(test_mission_registry mission_registry)
  # Loads the 3 shipped XML files straight from the source tree, not
  # install -- R14 style, the same files a real load() call will parse.
  target_compile_definitions(test_mission_registry PRIVATE
    "UAV_MISSION_CONFIG_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/config\"")

  # P9.4: fake navigator (7 in-process fake action servers) exercising the
  # real mission_executor_ros + broker (R20: fabricates rclcpp_action
  # traffic on real topic names, isolated domain).
  ament_add_gtest(test_mission_executor_node
    test/test_mission_executor_node.cpp TIMEOUT 180
    ENV ROS_DOMAIN_ID=98)
  target_link_libraries(test_mission_executor_node mission_executor_ros)
  target_compile_definitions(test_mission_executor_node PRIVATE
    "UAV_MISSION_CONFIG_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/config\"")
  ament_target_dependencies(test_mission_executor_node
    rclcpp rclcpp_action uav_interfaces nav_msgs diagnostic_msgs)

  # P9.4b: mission_executor_ros THROUGH the REAL control_authority_manager_
  # node in the same process (same shape as uav_safety's
  # test_safety_cut_chain.cpp) -- a probe seizes the SAFETY latch via the
  # real /control/set_authority service.
  ament_add_gtest(test_mission_authority_chain
    test/test_mission_authority_chain.cpp TIMEOUT 300
    ENV ROS_DOMAIN_ID=98)
  target_link_libraries(test_mission_authority_chain mission_executor_ros)
  target_compile_definitions(test_mission_authority_chain PRIVATE
    "UAV_MISSION_CONFIG_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/config\"")
  ament_target_dependencies(test_mission_authority_chain
    rclcpp rclcpp_action uav_interfaces nav_msgs diagnostic_msgs uav_control_authority)
endif()

ament_package()
