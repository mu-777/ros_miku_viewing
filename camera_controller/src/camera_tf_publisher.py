#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import Transform, TransformStamped, Quaternion, Vector3
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

DEFAULT_NODE_NAME = 'camera_tf_publisher'


class CameraTFPublisher(object):
    def __init__(self):
        self._dist2target = rospy.get_param('/camera_target_distance')
        self.pub_camera_tf = (TransformStamped(header=Header(stamp=rospy.Time.now(),
                                                             frame_id=rospy.get_param('/camera_focus_point_frame_id')),
                                               child_frame_id=rospy.get_param('/camera_frame_id')))

        self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._subscriber = rospy.Subscriber(rospy.get_param('/android_imu_topic'),
                                            Imu,
                                            self._android_imu_callback)

    def _android_imu_callback(self, imu):
        self.pub_camera_tf.header.stamp = rospy.Time.now()
        self.pub_camera_tf.transform.rotation = imu.orientation

    def publish_tf(self):
        self._tf_broadcaster.sendTransform(self.pub_camera_tf)

# --------------------------------------------
if __name__ == '__main__':
    rospy.init_node(DEFAULT_NODE_NAME, anonymous=True)
    rate_mgr = rospy.Rate(100)  # Hz

    camera_tf_pub = CameraTFPublisher()

    while not rospy.is_shutdown():
        camera_tf_pub.publish_tf()
        rate_mgr.sleep()




