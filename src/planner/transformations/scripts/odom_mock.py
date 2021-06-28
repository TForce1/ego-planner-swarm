#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import Image
from time import sleep

ODOM_TOPIC_NAME = "/odom"
IMAGE_TOPIC_NAME = "/camera/depth/image_rect_raw"

def send_odom(msg):
    global pub_odom, header

    msg_cmd = Odometry()
    msg_cmd.header = header
    msg_cmd.header.frame_id = "odom"
    pub_odom.publish(msg_cmd)
    #print(msg_cmd)

def ros_callback(msg):
    global pub_odom
    #print(msg.header)
    msg_cmd = Odometry()
    msg_cmd.header = msg.header
    msg_cmd.header.frame_id = "odom"
    pub_odom.publish(msg_cmd)
    
if __name__ == '__main__':
    global pub_odom
    rospy.init_node('odom_mock')

    pub_odom = rospy.Publisher(ODOM_TOPIC_NAME, Odometry, queue_size=1)
    rospy.Subscriber(IMAGE_TOPIC_NAME, Image, ros_callback, queue_size=1) 

    #timer = rospy.Timer(rospy.Duration(0.1), send_odom)

    rospy.spin()
