#!/usr/bin/env python

import argparse
import rospy
import math
import sys
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from tf.transformations import *
import tf


class OdomShift:

    NODE_NAME="odom_shift"
    ODOM_TOPIC="/odom_z"
    QUEUE_SIZE=100
    P = math.pi/2
    ROTATION_RPY = [0, 0, 0]

    def __init__(self, odometry_topic):
        rospy.init_node(self.NODE_NAME, anonymous=False)
        self.odom = odometry_topic
        self.sub = rospy.Subscriber(self.odom, Odometry, self.transform)
        self.pub = rospy.Publisher(self.ODOM_TOPIC, Odometry, queue_size=self.QUEUE_SIZE)
        self.rot_quat = quaternion_from_euler(self.ROTATION_RPY[0], self.ROTATION_RPY[1], self.ROTATION_RPY[2])

        rospy.loginfo("<<Odom Z Shifting>>: node: %s, input_topic: %s, output_topic: %s" % \
                      (self.NODE_NAME, self.odom, self.ODOM_TOPIC))
        self.br = tf.TransformBroadcaster()

    def transform(self, msg):
        orientation = msg.pose.pose.orientation
        odom_quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        rotated = self.quaternion_rotation(self.rot_quat, odom_quat)

        orientation_msg = Quaternion(rotated[0], rotated[1], rotated[2], rotated[3])
        odom_shift_msg = msg
        odom_shift_msg.pose.pose.position.z = 0.16 # 0.1
        odom_shift_msg.pose.pose.position.x=  -msg.pose.pose.position.x
        odom_shift_msg.pose.pose.position.y =  -msg.pose.pose.position.y

        odom_shift_msg.pose.pose.orientation = orientation_msg
        odom_shift_msg.header.frame_id = "world"
        self.pub.publish(odom_shift_msg)

        self.br.sendTransform(
            (odom_shift_msg.pose.pose.position.x, odom_shift_msg.pose.pose.position.y, 0.16),
            (rotated),
            rospy.Time.now(),
            "front_camera",
            "world")

    @staticmethod
    def quaternion_rotation(rot_quat, quat):
        _ , _, yaw1 = euler_from_quaternion(quat)
        # print("yaw1: ", yaw1)
        quat_mul = quaternion_multiply(quat, rot_quat)
        quat_norm = quat_mul / np.linalg.norm(quat_mul)
        _ , _, yaw = euler_from_quaternion(quat_norm)
        # print("yaw2: ", yaw)

        quat = quaternion_from_euler(0, 0, yaw)
        # print(quat / np.linalg.norm(quat))
        return quat / np.linalg.norm(quat)

if __name__ == '__main__':

    args = rospy.myargv(sys.argv)
    odom = args[1]

    try:
        odom_shift = OdomShift(odom)
        rospy.spin()
        pass

    except rospy.ROSInterruptException:
        rospy.logerr("ROSInterruptException was thrown from odom_shift")
