#!/usr/bin/env python

# /* ----------------------------------------------------------------------------
#  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Jesus Tordesillas, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import roslib
import rospy
import math
from snapstack_msgs.msg import Goal, State
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from quadrotor_msgs.msg import PositionCommand
import numpy as np
from numpy import linalg as LA

from tf.transformations import euler_from_quaternion

from pyquaternion import Quaternion
import tf
import math  


class GoalToCmdVel:

    def __init__(self):
        self.state=State()

        self.state.pos.x = rospy.get_param('~x', 0.0)
        self.state.pos.y = rospy.get_param('~y', 0.0)
        self.state.pos.z = rospy.get_param('~z', 0.0)

        self.state.quat.x = 0
        self.state.quat.y = 0
        self.state.quat.z = 0
        self.state.quat.w = 1

        self.current_yaw=0.0

        #Publishers
        self.pubCmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=1, latch=True)

        #Timers
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cmdVelCB)

        self.kv =1.0
        self.kdist = 0.2 #2.5
        self.kw =1.0
        self.kyaw = 0.5
        self.kalpha = 0.0

        self.state_initialized=False
        self.goal_initialized=False

        self.goal=PositionCommand()
        self.goal.position.x=0.0
        self.goal.position.y=0.0
        self.goal.position.z=0.0
        self.goal.velocity.x=0.0
        self.goal.velocity.y=0.0
        self.goal.velocity.z=0.0
        self.goal.acceleration.x=0.0
        self.goal.acceleration.y=0.0
        self.goal.acceleration.z=0.0

        self.state_initialized=False

    def odomCB(self, msg):
        self.state.pos.x = msg.pose.pose.position.x
        self.state.pos.y = msg.pose.pose.position.y
        self.state.pos.z = msg.pose.pose.position.z

        (yaw, _, _)=euler_from_quaternion((msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w), "szyx")
        
        self.current_yaw = yaw

        self.state.vel = msg.twist.twist.linear

        self.state.quat=msg.pose.pose.orientation

        self.state.w = msg.twist.twist.angular

        self.state_initialized=True

    def goalCB(self, goal):

        self.goal=goal

        self.goal_initialized=True


    def cmdVelCB(self, goal):
        if (self.state_initialized==False or self.goal_initialized==False):
            return

        twist=Twist()

        x = self.goal.position.x
        y = self.goal.position.y
        xd = self.goal.velocity.x
        yd = self.goal.velocity.y
        xd2 = self.goal.acceleration.x
        yd2 = self.goal.acceleration.y

        v_desired = math.sqrt(xd**2 + yd**2)
        alpha = self.current_yaw - math.atan2(y - self.state.pos.y, x - self.state.pos.x)
        alpha=self.wrapPi(alpha)
        forward=1

        if(alpha <= 3.14 / 2.0 and alpha > -3.14 / 2.0):
          forward=1
        else:
          forward=-1

        dist_error = forward * math.sqrt( (x - self.state.pos.x)**2 + (y - self.state.pos.y)**2  )

        # if (abs(dist_error)<0.03):
        #   alpha=0

        vel_norm=LA.norm(np.array([self.goal.velocity.x, self.goal.velocity.y]))
        if (abs(dist_error)<0.15 and vel_norm<0.02):   

            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:

            desired_yaw=math.atan2(yd,xd) 
            yaw_error = self.current_yaw - desired_yaw
            yaw_error=self.wrapPi(yaw_error)

            if math.isnan(self.goal.yaw_dot):
                print("yaw dot is nan")
                self.goal.yaw_dot = 0.0

            twist.linear.x = self.kv * v_desired + self.kdist * dist_error 
            twist.linear.x = max(0, min(0.25, twist.linear.x))
            twist.angular.z =   self.kyaw * yaw_error + self.kalpha * alpha - self.kw * self.goal.yaw_dot# + self.goal.dyaw
            twist.angular.z = max(-1.0, min(1.0, twist.angular.z))

        self.pubCmdVel.publish(twist)


    def wrapPi(self, x):
        x=(x+np.pi) % (2 * np.pi)
        if(x<0):
            x=x+2 * np.pi
        return x-np.pi   

    def printAngle(self, value, name):
        print (name, '{:.3f}'.format(value), " rad (",'{:.3f}'.format(value * 180 / 3.14), " deg) ")


if __name__ == '__main__':

    try:
        rospy.init_node('goal_to_cmd_vel')
        c = GoalToCmdVel()

        rospy.Subscriber("/odom_z", Odometry, c.odomCB, queue_size=1) 

        rospy.Subscriber("/drone_0_planning/pos_cmd", PositionCommand, c.goalCB, queue_size=1)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
