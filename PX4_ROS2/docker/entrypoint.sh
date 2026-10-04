#!/bin/bash
# Sources ROS and the workspace, then runs whatever was asked.
# `exec` matters: without it this shell stays as PID 1 and swallows the signals meant
# for the node, so `docker stop` would hard-kill instead of letting rosbag_manager_node
# close its bag cleanly (G-O1: metadata.yaml is only written on a clean close).
set -e
source /opt/ros/"${ROS_DISTRO}"/setup.bash
source /ws/install/setup.bash
exec "$@"
