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


class CameraRotate:

    NODE_NAME="camera_rotate"
    CAMERA_POSE_TOPIC="camera_pose"
    QUEUE_SIZE=100
    P = math.pi/2
    ROTATION_RPY = [-P, 0, -P]
#    ROTATION_RPY = [0, 0, 0]
    def __init__(self, odometry_topic):
        rospy.init_node(self.NODE_NAME, anonymous=False)
        self.odom = odometry_topic
        self.sub = rospy.Subscriber(self.odom, Odometry, self.transform)
        self.pub = rospy.Publisher(self.CAMERA_POSE_TOPIC, PoseStamped, queue_size=self.QUEUE_SIZE)
        self.rot_quat = quaternion_from_euler(self.ROTATION_RPY[0], self.ROTATION_RPY[1], self.ROTATION_RPY[2])
        rospy.loginfo("<<Camera Pose Rotation>>: node: %s, input_topic: %s, output_topic: %s" % \
                      (self.NODE_NAME, self.odom, self.CAMERA_POSE_TOPIC))

    def transform(self, msg):
        orientation = msg.pose.pose.orientation
        odom_quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        rotated = self.quaternion_rotation(self.rot_quat, odom_quat)
        orientation_msg = Quaternion(rotated[0], rotated[1], rotated[2], rotated[3])
        position_msg = Pose(position=msg.pose.pose.position, orientation=orientation_msg)
        camera_pose_msg = PoseStamped(header=msg.header, pose=position_msg)
        camera_pose_msg.pose.position.z = 0.16
        self.pub.publish(camera_pose_msg)
        

    @staticmethod
    def quaternion_rotation(rot_quat, quat):
        quat_mul = quaternion_multiply(quat, rot_quat)
        quat_norm = quat_mul / np.linalg.norm(quat_mul)
        return quat_norm

if __name__ == '__main__':

    args = rospy.myargv(sys.argv)
    odom = args[1]

    try:
        camera_rotate = CameraRotate(odom)
        rospy.spin()
        pass

    except rospy.ROSInterruptException:
        rospy.logerr("ROSInterruptException was thrown from camera_rotate")

