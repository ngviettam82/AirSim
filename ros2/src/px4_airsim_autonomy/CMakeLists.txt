cmake_minimum_required(VERSION 3.8)
project(px4_airsim_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Core Autonomy Algorithms Library
add_library(${PROJECT_NAME}_core
  src/algorithm_factory.cpp
  src/enterprise/enterprise_flight_supervisor.cpp
  src/enterprise/photogrammetry_survey_mission.cpp
  src/enterprise/dynamic_avoidance_mission.cpp
  src/production/smart_rth_battery.cpp
  src/production/geofence_3d.cpp
  src/production/adsb_deconfliction.cpp
  src/production/photogrammetry_calc.cpp
  src/production/boustrophedon_planner.cpp
  src/production/mission_continuity.cpp
  src/production/dynamic_stopping_bubble.cpp
  src/production/safe_flight_corridor.cpp
  src/production/quadcopter_spin_recovery.cpp
  src/production/multi_camera_depth.cpp
  src/production/degraded_navigation_fsm.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
)

# Core Autonomy Flight Mode Executable
add_executable(autonomous_flight_mode_node
  src/autonomous_flight_mode_node.cpp
)

target_link_libraries(autonomous_flight_mode_node
  ${PROJECT_NAME}_core
  Eigen3::Eigen
)

ament_target_dependencies(autonomous_flight_mode_node
  rclcpp
  px4_ros2_cpp
  px4_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
)

# Unit Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_algorithms
    test/test_algorithms.cpp
  )
  target_link_libraries(test_algorithms
    ${PROJECT_NAME}_core
    Eigen3::Eigen
  )
  ament_target_dependencies(test_algorithms
    rclcpp
    geometry_msgs
    sensor_msgs
  )

  ament_add_gtest(test_production_systems
    test/test_production_systems.cpp
  )
  target_link_libraries(test_production_systems
    ${PROJECT_NAME}_core
    Eigen3::Eigen
  )
  ament_target_dependencies(test_production_systems
    rclcpp
    geometry_msgs
    sensor_msgs
  )
endif()

install(TARGETS
  ${PROJECT_NAME}_core
  autonomous_flight_mode_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  Eigen3
)

ament_package()

