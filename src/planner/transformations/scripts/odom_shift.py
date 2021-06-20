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


class OdomShift:

    NODE_NAME="odom_shift"
    ODOM_TOPIC="/odom_z"
    QUEUE_SIZE=100
    P = math.pi/2
    ROTATION_RPY = [-P, 0, -P]

    def __init__(self, odometry_topic):
        rospy.init_node(self.NODE_NAME, anonymous=False)
        self.odom = odometry_topic
        self.sub = rospy.Subscriber(self.odom, Odometry, self.transform)
        self.pub = rospy.Publisher(self.ODOM_TOPIC, Odometry, queue_size=self.QUEUE_SIZE)
        rospy.loginfo("<<Odom Z Shifting>>: node: %s, input_topic: %s, output_topic: %s" % \
                      (self.NODE_NAME, self.odom, self.ODOM_TOPIC))

    def transform(self, msg):
        odom_shift_msg = msg
        odom_shift_msg.pose.pose.position.z = 1.0

        self.pub.publish(odom_shift_msg)
        print(odom_shift_msg)


if __name__ == '__main__':

    args = rospy.myargv(sys.argv)
    odom = args[1]

    try:
        odom_shift = OdomShift(odom)
        rospy.spin()
        pass

    except rospy.ROSInterruptException:
        rospy.logerr("ROSInterruptException was thrown from odom_shift")
