#!/usr/bin/python3
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.time import CONVERSION_CONSTANT
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose


MAX_NO_OF_DETECTIONS = 3
DETECTION_COUNT_PERIOD = 5.0
TF_PUBLISH_PERIOD = 0.2
TF_RANGE = 1.0
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

    def _on_tf(self):
        t = self.get_clock().now()

        tf = TransformStamped()
        tf.header.frame_id = 'map'
        tf.header.stamp = t.to_msg()
        tf.child_frame_id = 'sensor'

        t_s = t.nanoseconds / CONVERSION_CONSTANT
        tf.transform.translation.x = math.cos(t_s) * TF_RANGE
        tf.transform.translation.y = math.sin(t_s) * TF_RANGE
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
            d.id = f'd{i}'
            d.bbox.size.x = 1.0
            d.bbox.size.y = 0.5
            d.bbox.size.z = 0.2
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = '1'
            h.hypothesis.score = 1.0
            h.pose.pose.position.x = math.cos(t_s) * i
            h.pose.pose.position.y = math.sin(t_s) * i
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
