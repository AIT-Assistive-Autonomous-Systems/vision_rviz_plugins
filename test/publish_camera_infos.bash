#!/bin/bash
set -eu
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

ros2 run tf2_ros static_transform_publisher 0 3 3 0 0 1.92 map test_optical_frame &
ros2 topic pub /camera_info sensor_msgs/msg/CameraInfo "$(cat $SCRIPT_DIR/test_camera_info.yaml)" &
ros2 topic pub /camera_info_roi sensor_msgs/msg/CameraInfo "$(cat $SCRIPT_DIR/test_camera_info_roi.yaml)"
