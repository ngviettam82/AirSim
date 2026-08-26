cmake_minimum_required(VERSION 3.8)
project(uav_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

set(msg_files
  "msg/ResultCode.msg"
  "msg/VehicleState.msg"
  "msg/ControlAuthority.msg"
  "msg/ControlCommand.msg"
  "msg/SafetyState.msg"
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory3D.msg"
  "msg/Path3D.msg"
  "msg/AvoidanceAdvice.msg"
  "msg/OffboardStatus.msg"
  "msg/MarkerObservation.msg"
  "msg/TargetTrack.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/SemanticLandmark.msg"
  "msg/SemanticLandmarkArray.msg"
  "msg/TargetState.msg"
  "msg/MissionStatus.msg"
  "msg/MissionEvent.msg"
  "msg/LocalizationStatus.msg"
  "msg/VehicleHealth.msg"
)

set(srv_files
  "srv/Arm.srv"
  "srv/Disarm.srv"
  "srv/SetFlightMode.srv"
  "srv/SetControlAuthority.srv"
  "srv/LoadMission.srv"
  "srv/PauseMission.srv"
  "srv/ResumeMission.srv"
  "srv/AbortMission.srv"
  "srv/ClearFault.srv"
)

set(action_files
  "action/Takeoff.action"
  "action/Land.action"
  "action/GotoPose.action"
  "action/HoldPosition.action"
  "action/FollowPath.action"
  "action/TrackTarget.action"
  "action/InspectMarker.action"
  "action/Recover.action"
  "action/ExecuteMission.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
    action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Reads the .msg/.srv/.action text; nothing is generated, built or spun up. Eleven
  # packages depend on these definitions, so a convention broken here reaches all of them
  # while still compiling fine.
  ament_add_pytest_test(test_interface_conventions test/test_interface_conventions.py
                        TIMEOUT 60)
endif()

ament_package()
