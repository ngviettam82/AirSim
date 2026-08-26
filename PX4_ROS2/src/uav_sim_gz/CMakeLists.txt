cmake_minimum_required(VERSION 3.8)
project(uav_sim_gz)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Belongs to sim-world layer, not uav_perception; see docs/package-status.md Debt #10.
add_executable(image_rate_probe src/image_rate_probe.cpp)
ament_target_dependencies(image_rate_probe rclcpp sensor_msgs)
install(TARGETS image_rate_probe DESTINATION lib/${PROJECT_NAME})

# Generated, not committed: reproducible, no licence to track.
set(GROUND_ASSETS ${CMAKE_CURRENT_BINARY_DIR}/ground_assets)
set(GROUND_STAMP ${GROUND_ASSETS}/textures/ground_rich.png)

# 400 m default; repeating-UV mesh costs nothing extra to run.
set(GROUND_EXTENT_M 400 CACHE STRING "Side of the generated ground plane, metres")

add_custom_command(
  OUTPUT ${GROUND_STAMP}
  COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_ground_texture.py
          --out ${GROUND_ASSETS} --extent ${GROUND_EXTENT_M}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_ground_texture.py
  COMMENT "Generating ground textures and meshes"
  VERBATIM
)
add_custom_target(ground_assets ALL DEPENDS ${GROUND_STAMP})

# PYTHONNOUSERSITE avoids the pip/apt numpy conflict; see README pitfall 9.
set(MARKER_ASSETS ${CMAKE_CURRENT_BINARY_DIR}/marker_models)
set(MARKER_STAMP ${MARKER_ASSETS}/aruco_7/model.sdf)

add_custom_command(
  OUTPUT ${MARKER_STAMP}
  COMMAND ${CMAKE_COMMAND} -E env PYTHONNOUSERSITE=1 ${Python3_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_aruco_marker.py
          --out ${MARKER_ASSETS}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_aruco_marker.py
  COMMENT "Generating ArUco marker boards"
  VERBATIM
)
add_custom_target(marker_assets ALL DEPENDS ${MARKER_STAMP})

install(
  DIRECTORY models worlds airframes config launch docs tools
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY ${GROUND_ASSETS}/
  DESTINATION share/${PROJECT_NAME}/models/ground_textured/materials
)

install(
  DIRECTORY ${MARKER_ASSETS}/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  PROGRAMS scripts/install_to_px4.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Gazebo resolves model:// and world paths through this variable.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gz_resource_path.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Pure XML parsing: no ROS graph, no Gazebo, no /clock, so no ROS_DOMAIN_ID is needed
  # (R20 allocates domains only to tests that create a participant). Runs in under a
  # second, which is why the R0 invariant it guards can afford to be checked every build.
  ament_add_pytest_test(test_sdf_invariants test/test_sdf_invariants.py TIMEOUT 60)
endif()

ament_package()
