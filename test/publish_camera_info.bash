#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0.78525 map test_optical_frame &
ros2 topic pub /camera_info sensor_msgs/msg/CameraInfo "$(cat $SCRIPT_DIR/test_camera_info.yaml)"