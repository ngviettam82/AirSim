cmake_minimum_required(VERSION 3.8)
project(uav_observability)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# Declared for the node layer to follow (P10.2/P10.4/P10.5) -- not used by
# the 3 pure libs below, same split as uav_safety's failsafe_policy/rclcpp.
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
# P10.2: real compression (SequentialCompressionWriter), not a stub -- G-O1
# (P10.3) picks the actual mode/format empirically.
find_package(rosbag2_compression REQUIRED)
# P10.8c: Rosbag2QoS YAML::convert -- writes offered_qos_profiles with the
# SAME encoder `ros2 bag play` uses to decode it back (bug fix, not a stub).
find_package(rosbag2_transport REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# P10.5: MissionEvent/MissionStatus/SafetyState/ControlAuthority/VehicleState/
# OffboardStatus/LocalizationStatus typed subs -- internal interfaces only,
# never px4_msgs (R1).
find_package(uav_interfaces REQUIRED)

# Disk-budget arithmetic (P10 plan S:2.1) -- ROS-free, same rule as
# uav_safety's failsafe_policy: a safety/evidence-critical matrix is pinned
# by a test, not by a flight. Nothing here pulls rclcpp or any px4_msgs (R1).
add_library(bag_retention SHARED src/bag_retention.cpp)
target_include_directories(bag_retention PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Freshness board (P10 plan S:2.2, O4/R32) -- ROS-free.
add_library(staleness_board SHARED src/staleness_board.cpp)
target_include_directories(staleness_board PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Edge-triggered timeline formatter (P10 plan S:2.3, O2) -- ROS-free.
add_library(event_ledger SHARED src/event_ledger.cpp)
target_include_directories(event_ledger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Split from the executable so one test process can drive the real node
# (same shape as uav_safety's safety_supervisor_ros).
add_library(rosbag_manager_ros SHARED src/rosbag_manager_node.cpp)
target_include_directories(rosbag_manager_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rosbag_manager_ros bag_retention)
ament_target_dependencies(rosbag_manager_ros
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rosbag2_compression
  rosbag2_transport
  diagnostic_msgs
)

add_executable(rosbag_manager_node src/rosbag_manager_main.cpp)
target_link_libraries(rosbag_manager_node rosbag_manager_ros)
ament_target_dependencies(rosbag_manager_node rclcpp)

# P10.4: sub A (GenericSubscription cadence board) + sub B (typed
# DiagnosticArray content) -- ROS deps only rclcpp/diagnostic_msgs, same
# split-from-executable shape as rosbag_manager_ros.
add_library(diagnostics_node_ros SHARED src/diagnostics_node.cpp)
target_include_directories(diagnostics_node_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(diagnostics_node_ros staleness_board)
# P10.9b: phase predicate reads uav_interfaces/msg/VehicleState.armed off a
# dedicated typed subscription (R1: internal interface only, never px4_msgs).
ament_target_dependencies(diagnostics_node_ros rclcpp diagnostic_msgs uav_interfaces)

add_executable(diagnostics_node src/diagnostics_main.cpp)
target_link_libraries(diagnostics_node diagnostics_node_ros)
ament_target_dependencies(diagnostics_node rclcpp)

# P10.5: typed sub on 10 sources -> EventLedger -> JSONL file (plan S:2.3).
# ROS deps: rclcpp, diagnostic_msgs (safety/violations, diagnostics/aggregated
# sources + own diagnostics), std_msgs (state/estimator_source), uav_interfaces
# (the other 7 typed sources). Same split-from-executable shape as the other
# two P10 nodes.
add_library(event_logger_node_ros SHARED src/event_logger_node.cpp)
target_include_directories(event_logger_node_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(event_logger_node_ros event_ledger)
ament_target_dependencies(event_logger_node_ros rclcpp diagnostic_msgs std_msgs uav_interfaces)

add_executable(event_logger_node src/event_logger_main.cpp)
target_link_libraries(event_logger_node event_logger_node_ros)
ament_target_dependencies(event_logger_node rclcpp)

install(TARGETS rosbag_manager_node diagnostics_node event_logger_node
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS
    bag_retention staleness_board event_ledger
    rosbag_manager_ros diagnostics_node_ros event_logger_node_ros
  EXPORT export_uav_observability
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_uav_observability HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(
  bag_retention staleness_board event_ledger
  rosbag_manager_ros diagnostics_node_ros event_logger_node_ros)
ament_export_dependencies(
  rclcpp rosbag2_cpp rosbag2_storage rosbag2_compression diagnostic_msgs std_msgs uav_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # No ROS_DOMAIN_ID env needed (R20): no rclcpp::init, no domain participant.
  ament_add_gtest(test_bag_retention test/test_bag_retention.cpp)
  target_link_libraries(test_bag_retention bag_retention)

  ament_add_gtest(test_staleness_board test/test_staleness_board.cpp)
  target_link_libraries(test_staleness_board staleness_board)

  ament_add_gtest(test_event_ledger test/test_event_ledger.cpp)
  target_link_libraries(test_event_ledger event_ledger)

  # This probe fabricates traffic on real topics (R20): isolated domain.
  ament_add_gtest(test_rosbag_manager_node
    test/test_rosbag_manager_node.cpp TIMEOUT 180
    ENV ROS_DOMAIN_ID=99)
  target_link_libraries(test_rosbag_manager_node rosbag_manager_ros bag_retention)
  ament_target_dependencies(test_rosbag_manager_node
    rclcpp rosbag2_cpp rosbag2_storage rosbag2_transport diagnostic_msgs std_msgs)

  # This probe fabricates traffic on real topics (R20): isolated domain.
  ament_add_gtest(test_diagnostics_node
    test/test_diagnostics_node.cpp TIMEOUT 180
    ENV ROS_DOMAIN_ID=99)
  target_link_libraries(test_diagnostics_node diagnostics_node_ros staleness_board)
  ament_target_dependencies(test_diagnostics_node rclcpp diagnostic_msgs std_msgs uav_interfaces)
  # P10.9b: ShippedWaiverYamlMatchesRealSafetyChildNames loads THIS package's
  # own checked-in config/preflight_waivers.yaml straight off source (not an
  # installed share path -- there is no ament_index_cpp dependency here) to
  # catch P8/uav_safety child-name drift at test time.
  target_compile_definitions(test_diagnostics_node PRIVATE
    UAV_OBSERVABILITY_PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  # This probe fabricates traffic on real topics (R20): isolated domain.
  ament_add_gtest(test_event_logger_node
    test/test_event_logger_node.cpp TIMEOUT 180
    ENV ROS_DOMAIN_ID=99)
  target_link_libraries(test_event_logger_node event_logger_node_ros event_ledger)
  ament_target_dependencies(test_event_logger_node rclcpp diagnostic_msgs std_msgs uav_interfaces)
endif()

ament_package()
