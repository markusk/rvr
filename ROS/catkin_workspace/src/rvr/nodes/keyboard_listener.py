#!/usr/bin/env python3
# coding=utf-8

"""
Simple keyboard_listener demo that listens to geometry_msgs/Twist
published by the 'cmd_vel' topic of the teleop_twist_keyboard node
and prints out the received data/messsage.

That's it.


Usage:
roslaunch rvr keyboard_control.launch

Author:  Markus Knapp, 2020
Website: https://direcs.de
"""


import rospy
from geometry_msgs.msg import Twist


def callback(data):
    # print out received message from the teleop_twist_keyboard
    rospy.loginfo(rospy.get_caller_id() + ' x=%s\n', data.linear.x)
    rospy.loginfo(rospy.get_caller_id() + ' y=%s\n', data.linear.y)
    rospy.loginfo(rospy.get_caller_id() + ' z=%s\n', data.angular.z)


def listener():
    # In ROS, nodes are uniquely named. The anonymous=True flag
    # means that rospy will choose a unique name for this listener node
    rospy.init_node('keyboard_listener', anonymous=True)

    # subscribe the message cmd_vel
    rospy.Subscriber('cmd_vel', Twist, callback)

    # Ready
    rospy.loginfo("Ready. Control me via keyboard now.")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
