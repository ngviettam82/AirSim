cmake_minimum_required(VERSION 3.8)
project(uav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  test/smoke_flight.py
  test/g2_fused_accuracy.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Static only: builds both LaunchDescriptions in-process and evaluates conditions
  # against the declared defaults. No ROS graph, no processes, no ROS_DOMAIN_ID (R20),
  # about a third of a second. The live half -- watching the reviewed:= guard engage for
  # real -- stays in scripts/verify_real_launch_inert.sh, which starts 18 nodes and would
  # not belong in a suite that already has load-sensitive timing tests.
  ament_add_pytest_test(test_launch_static test/test_launch_static.py TIMEOUT 60)
  # M5's safety verdict, unit-tested without a simulator on purpose: a rule that can
  # only be exercised by flying is a rule nobody exercises.
  ament_add_pytest_test(test_smoke_flight_safety_verdict
                        test/test_smoke_flight_safety_verdict.py TIMEOUT 60)
endif()

ament_package()
