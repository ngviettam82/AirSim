# syntax=docker/dockerfile:1.7
#
# ONE Dockerfile for both architectures (P11.1). amd64 is the workstation/sim image;
# arm64 is the companion-computer image that will fly on the aircraft.
#
# WHY ONE FILE AND NOT TWO. The sim-to-real gap this project is most exposed to is not
# aerodynamic, it is SYSTEMS: the aircraft's companion computer running a different
# library set from the one every gate was measured against. Two Dockerfiles drift
# silently and nobody notices until the drone is in the air. One file with an explicit
# PROFILE argument makes every difference between sim and aircraft a line you can read.
#
# THE ONE DIFFERENCE, AND WHY IT IS LEGITIMATE. uav_sim_gz is the simulated world plus
# the Gazebo->ROS bridge. CLAUDE.md S:4.11 states outright that it has no counterpart on
# the real aircraft -- it IS the simulator. So the aircraft profile builds the other 11
# packages and never pulls Gazebo. That is not a parity violation; shipping Gazebo to a
# flight computer would be.
#
#   PROFILE=aircraft  (default)  11 packages, no Gazebo. This is what flies.
#   PROFILE=sim                  all 12 + ros_gz. Workstation only.
#
# ARCHITECTURE TARGET (project owner, 2026-08-25): GENERIC arm64 on Ubuntu 22.04 /
# ros:humble, deliberately NOT a Jetson L4T base. No companion computer has been chosen
# yet, and an L4T base pins a JetPack version that a not-yet-purchased board may not run.
# Generic arm64 runs on Jetson, Pi and Orange Pi alike; the cost is no CUDA, so DNN
# perception runs on CPU. Moving to JetPack later changes the FROM line and the CUDA
# packages, not the structure of this file.
#
# 🔴 WHAT THIS IMAGE DELIBERATELY DOES NOT CARRY.
#   - No fastdds_large_samples.xml. That profile is sim-only until P11.3 measures it on
#     the target hardware (see scripts/check_dds_profile_sim_only.sh, which enforces it).
#   - No sensor noise-injection parameters. Flying with simulated sensors enabled is a
#     scenario that must not be constructible (debt #9).
#
# Build: bash docker/build.sh            (see that script for buildx/QEMU setup)

ARG ROS_DISTRO=humble

# Two stages on purpose. `deps` ends once every dependency is installed, which is where
# cross-architecture portability actually breaks (a package with no arm64 build). `build`
# then compiles. Under QEMU the compile costs an order of magnitude more than the deps,
# so being able to prove the cheap half separately is worth one extra FROM line --
# docker/build.sh targets `deps` for the arm64 smoke check.
FROM ros:${ROS_DISTRO}-ros-base AS deps
ARG ROS_DISTRO
ARG PROFILE=aircraft
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# BehaviorTree.CPP v3 is pinned by uav_mission (v3.8, not v4 -- the XML dialect differs).
# python3-colcon-common-extensions is not in ros-base.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      git \
      python3-colcon-common-extensions \
      python3-rosdep \
      ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

# The workspace's external ROS SOURCE dependencies -- both are packages, not rosdep
# keys, so `rosdep install` cannot supply them and reports
# "Cannot locate rosdep definition for [px4_ros_com]" if they are missing. That is how
# this list was established: the first build of this file failed on exactly that, which
# is worth more than reading package.xml, because it proves the list is complete.
#
# Pinned to the branch matching the flight stack this project is verified against, PX4
# v1.15.4. An unpinned clone would silently change the /fmu/* message definitions under
# a stack whose entire safety argument rests on them.
ARG PX4_MSGS_REF=release/1.15
ARG PX4_ROS_COM_REF=release/1.15
RUN git clone --depth 1 --branch ${PX4_MSGS_REF} \
      https://github.com/PX4/px4_msgs.git src/px4_msgs \
 && git clone --depth 1 --branch ${PX4_ROS_COM_REF} \
      https://github.com/PX4/px4_ros_com.git src/px4_ros_com

COPY src/ src/

# uav_sim_gz is the simulator itself; the aircraft image must not contain it, and must
# not drag ros_gz in through its dependencies.
RUN if [ "${PROFILE}" = "aircraft" ]; then rm -rf src/uav_sim_gz; fi

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update \
    && (rosdep update --rosdistro ${ROS_DISTRO} || true) \
    && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

FROM deps AS build
ARG ROS_DISTRO
ARG PROFILE=aircraft
SHELL ["/bin/bash", "-c"]
WORKDIR /ws

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Fail the build rather than ship a flight image with the simulator in it. A grep is a
# weak check in general, but here the thing being excluded is a whole package directory,
# so its absence is unambiguous.
RUN if [ "${PROFILE}" = "aircraft" ] && [ -d install/share/uav_sim_gz ]; then \
      echo "FATAL: uav_sim_gz present in an aircraft image"; exit 1; fi

ENV PROFILE=${PROFILE}
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
