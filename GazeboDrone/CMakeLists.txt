cmake_minimum_required(VERSION 3.12.0 FATAL_ERROR)

project(GazeboDrone)

set(AIRSIM_ROOT ..)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(gazebo REQUIRED)

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${AIRSIM_ROOT}/AirLib/deps/eigen3
    ${AIRSIM_ROOT}/AirLib/deps/rpclib/include
    ${AIRSIM_ROOT}/AirLib/include
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ${AIRSIM_ROOT}/build_release/output/lib
    ${AIRSIM_ROOT}/AirLib/lib
    ${AIRSIM_ROOT}/AirLib/deps/rpclib/lib
    ${AIRSIM_ROOT}/AirLib/deps/MavLinkCom/lib
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(
    ${PROJECT_NAME} 
    ${GAZEBO_LIBRARIES}
    pthread
    AirLib
    rpc
    tbb
    stdc++fs
)

