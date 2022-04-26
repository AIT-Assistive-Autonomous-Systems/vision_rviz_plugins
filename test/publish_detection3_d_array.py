#!/usr/bin/python3
"""
Copyright 2021 Austrian Institute of Technology GmbH

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import math
import sys
import rclpy
from random import random
from rclpy.node import Node
from rclpy.time import CONVERSION_CONSTANT
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose


MAX_NO_OF_DETECTIONS = 3
DETECTION_COUNT_PERIOD = 10.0
TF_PUBLISH_PERIOD = 0.2
TF_RANGE = 2.0
DETECTION_DISTANCE = 1.5
DETECTION_PUBLISH_PERIOD = 0.1
POS_STDDEV = 0.5
ROT_STDDEV = 0.05


COV_POS_X_IDX = 6 * 0 + 0
COV_POS_Y_IDX = 6 * 1 + 1
COV_POS_Z_IDX = 6 * 2 + 2
COV_ROT_X_IDX = 6 * 3 + 3
COV_ROT_Y_IDX = 6 * 4 + 4
COV_ROT_Z_IDX = 6 * 5 + 5


class Detection3DArrayPublisher(Node):

    def __init__(self):
        super().__init__('detection3_d_array_publisher')
        self._tf_broadcaster = TransformBroadcaster(self, 1)
        self._tf_timer = self.create_timer(TF_PUBLISH_PERIOD, self._on_tf)
        self._pub_detections = self.create_publisher(Detection3DArray, 'detections', 1)
        self._detections_timer = self.create_timer(DETECTION_PUBLISH_PERIOD, self._on_detections)

        self._x_scale = [(random() - 0.5) * 0.8 for _ in range(MAX_NO_OF_DETECTIONS)]
        self._y_scale = [(random() - 0.5) * 0.8 for _ in range(MAX_NO_OF_DETECTIONS)]
        self._z_scale = [(random() - 0.5) * 0.8 for _ in range(MAX_NO_OF_DETECTIONS)]

    def _on_tf(self):
        t = self.get_clock().now()

        tf = TransformStamped()
        tf.header.frame_id = 'map'
        tf.header.stamp = t.to_msg()
        tf.child_frame_id = 'sensor'

        t_s = t.nanoseconds / CONVERSION_CONSTANT
        tf.transform.translation.x = math.cos(t_s) * TF_RANGE
        tf.transform.translation.y = math.sin(t_s) * TF_RANGE
        tf.transform.rotation.y = 0.707002
        tf.transform.rotation.w = 0.707002
        self._tf_broadcaster.sendTransform(tf)

    def _on_detections(self):
        msg = Detection3DArray()
        msg.header.frame_id = 'sensor'
        t = self.get_clock().now()
        msg.header.stamp = t.to_msg()

        t_s = t.nanoseconds / CONVERSION_CONSTANT
        timed_scale = math.sin(t_s / DETECTION_COUNT_PERIOD * math.tau)
        # publish varying number of detections to test dis- and reappearing visuals
        no_of_det = int(round(math.fabs(MAX_NO_OF_DETECTIONS * timed_scale)))
        for i in range(no_of_det):
            d = Detection3D()
            if i > 0:
                # publish first without id, to test text visibility in the plugin
                d.id = f'd{i}'
            d.bbox.size.x = 1.0 + self._x_scale[i]
            d.bbox.size.y = 1.0 + self._y_scale[i]
            d.bbox.size.z = 1.0 + self._z_scale[i]
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = '1'
            h.hypothesis.score = 1.0
            h.pose.pose.position.z = math.cos(t_s) * i * DETECTION_DISTANCE
            h.pose.covariance[COV_POS_X_IDX] = POS_STDDEV * POS_STDDEV
            h.pose.covariance[COV_POS_Y_IDX] = POS_STDDEV * POS_STDDEV
            h.pose.covariance[COV_POS_Z_IDX] = POS_STDDEV * POS_STDDEV
            h.pose.covariance[COV_ROT_X_IDX] = ROT_STDDEV * ROT_STDDEV
            h.pose.covariance[COV_ROT_Y_IDX] = ROT_STDDEV * ROT_STDDEV
            h.pose.covariance[COV_ROT_Z_IDX] = ROT_STDDEV * ROT_STDDEV
            d.results.append(h)
            msg.detections.append(d)
        self._pub_detections.publish(msg)


def main(args=sys.argv):
    rclpy.init(args=args)
    try:
        rclpy.spin(Detection3DArrayPublisher())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
