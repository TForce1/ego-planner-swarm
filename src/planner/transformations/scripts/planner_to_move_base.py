#!/usr/bin/env python

import argparse
import rospy
import math
import sys
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from quadrotor_msgs.msg import PositionCommand
from tf.transformations import *


class PosCmd:

    NODE_NAME="planner_to_move_base"
    OUTPUT_TOPIC="/move_base_simple/goal"
    QUEUE_SIZE=100

    def __init__(self, pos_cmd_topic, client):
        rospy.init_node(self.NODE_NAME, anonymous=False)
        self.pos_cmd = pos_cmd_topic
        self.sub = rospy.Subscriber(self.pos_cmd, PositionCommand, self.transform)
        self.pub = rospy.Publisher(self.OUTPUT_TOPIC, PoseStamped, queue_size=self.QUEUE_SIZE)
        print(self.sub)


        self.client = client
        self.client.wait_for_server()

        rospy.loginfo("<<Odom Z Shifting>>: node: %s, input_topic: %s, output_topic: %s" % \
                (self.NODE_NAME, self.pos_cmd, self.OUTPUT_TOPIC))


    def transform(self, msg):
        quat = quaternion_from_euler(0, 0, msg.yaw)
        orientation_msg = Quaternion(*quat)
        position_msg = Pose(position=msg.position, orientation=orientation_msg)
        # self.pub.publish(position_msg)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position = msg.position
        goal.target_pose.pose.orientation = orientation_msg
        self.client.send_goal(goal)
        # self.client.wait_for_result()
        state = self.client.get_state()
        print (state)



if __name__ == '__main__':

    args = rospy.myargv(sys.argv)
    pos_cmd = args[1]

    try:
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        odom_shift = PosCmd(pos_cmd, client)
        rospy.spin()
        pass

    except rospy.ROSInterruptException:
        rospy.logerr("ROSInterruptException was thrown from odom_shift")
