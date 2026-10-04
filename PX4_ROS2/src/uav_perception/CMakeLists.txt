cmake_minimum_required(VERSION 3.8)
project(uav_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco objdetect)

# How a camera stream fails. Separate from the node so the delivery arithmetic
# can be tested without a running system - and it is the arithmetic that matters,
# because a stream losing most of its frames still looks like a slower camera.
add_library(camera_health_checks SHARED src/camera_health_checks.cpp)
target_include_directories(camera_health_checks PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(camera_health_node src/camera_health_node.cpp)
target_link_libraries(camera_health_node camera_health_checks)
ament_target_dependencies(camera_health_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
)

# Marker pose arithmetic, above all the optical/body frame convention. Kept free
# of OpenCV and ROS types so the frame tests pin plain numbers: both frames are
# three doubles, so a swap compiles cleanly and only a test catches it.
add_library(marker_pose SHARED src/marker_pose.cpp)
target_include_directories(marker_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(marker_detector_node src/marker_detector_node.cpp)
target_link_libraries(marker_detector_node marker_pose ${OpenCV_LIBS})
ament_target_dependencies(marker_detector_node
  rclcpp
  sensor_msgs
  uav_interfaces
  cv_bridge
)

# Depth backprojection + flood-fill clustering; ROS/OpenCV-free like marker_pose.
# Links marker_pose for its pinned/tested opticalToBody() -- the ground
# filter's attitude compensation reuses that permutation rather than
# duplicating it (see obstacle_extraction.cpp).
add_library(obstacle_extraction SHARED src/obstacle_extraction.cpp)
target_include_directories(obstacle_extraction PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(obstacle_extraction marker_pose)

add_executable(obstacle_extractor_node src/obstacle_extractor_node.cpp)
target_link_libraries(obstacle_extractor_node obstacle_extraction ${OpenCV_LIBS})
ament_target_dependencies(obstacle_extractor_node
  rclcpp
  sensor_msgs
  uav_interfaces
  cv_bridge
  nav_msgs
)

# Per-axis constant-velocity Kalman + NN association; source-neutral like the
# other libs (obstacle_extractor_node is the only caller today, see README).
# Stays frame-neutral on purpose (see file header comment) -- ego_motion.cpp
# below does the optical<->odom conversion OUTSIDE this library.
add_library(target_tracking SHARED src/target_tracking.cpp)
target_include_directories(target_tracking PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Ego-motion compensation (bug #10 follow-up, 2026-08-23): optical<->odom
# point/vector transforms, ROS-free like marker_pose/obstacle_extraction.
# Links marker_pose for opticalToBody()/bodyToOptical()/AttitudeQuaternion.
add_library(ego_motion SHARED src/ego_motion.cpp)
target_include_directories(ego_motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ego_motion marker_pose)

add_executable(target_tracker_node src/target_tracker_node.cpp)
target_link_libraries(target_tracker_node target_tracking ego_motion marker_pose)
ament_target_dependencies(target_tracker_node
  rclcpp
  uav_interfaces
  nav_msgs
)

# Split from the executable so one test process can drive the real node
# (mirrors uav_world_model). Skeleton only - see README (plan P5 Sec 1/P5.7).
add_library(object_detector_ros SHARED src/object_detector_node.cpp)
target_include_directories(object_detector_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(object_detector_ros ${OpenCV_LIBS})
ament_target_dependencies(object_detector_ros
  rclcpp
  sensor_msgs
  vision_msgs
  cv_bridge
)

add_executable(object_detector_node src/object_detector_main.cpp)
target_link_libraries(object_detector_node object_detector_ros)
ament_target_dependencies(object_detector_node rclcpp)

install(TARGETS camera_health_node marker_detector_node obstacle_extractor_node
  target_tracker_node object_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS camera_health_checks marker_pose obstacle_extraction target_tracking
  ego_motion object_detector_ros
  EXPORT export_camera_health_checks
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_camera_health_checks HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(
  camera_health_checks marker_pose obstacle_extraction target_tracking ego_motion
  object_detector_ros)
ament_export_dependencies(
  rclcpp sensor_msgs diagnostic_msgs vision_msgs uav_interfaces cv_bridge nav_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_camera_health_checks test/test_camera_health_checks.cpp)
  target_link_libraries(test_camera_health_checks camera_health_checks)

  ament_add_gtest(test_marker_pose test/test_marker_pose.cpp)
  target_link_libraries(test_marker_pose marker_pose)

  ament_add_gtest(test_obstacle_extraction test/test_obstacle_extraction.cpp)
  target_link_libraries(test_obstacle_extraction obstacle_extraction marker_pose)

  ament_add_gtest(test_target_tracking test/test_target_tracking.cpp)
  target_link_libraries(test_target_tracking target_tracking ego_motion marker_pose)

  # Keeps the probe's fabricated camera stream off a live domain.
  ament_add_gtest(test_object_detector_node test/test_object_detector_node.cpp TIMEOUT 120
    ENV ROS_DOMAIN_ID=93)
  target_link_libraries(test_object_detector_node object_detector_ros ${OpenCV_LIBS})
  ament_target_dependencies(test_object_detector_node rclcpp sensor_msgs vision_msgs cv_bridge)
endif()

ament_package()
