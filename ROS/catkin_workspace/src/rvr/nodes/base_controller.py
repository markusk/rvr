#!/usr/bin/env python3
# coding=utf-8

"""
This is the ROS node for the RVR (https://direcs.de).

It expects "cmd_vel" geometry_msgs/Twist messages to control the robots motors.
It will then publish messages like "FORWARD, BACKWARD, LEFT, RIGHT, STOP" with the speed,
which will be received by a "motor_server" node. The latter is responsible for
controlling the motors with lowlevel I2C commands on a Raspberry Pi.

This node can (also) be controlled via keyboard with the
teleop_twist_keyboard or teleop_twist_joy node.

Usage:
roslaunch rvr keyboard_motor_control.launch
or
roslaunch rvr teleop_joy.launch


Author:  Markus Knapp, 2020
Website: https://direcs.de
"""


import rospy
from geometry_msgs.msg import Twist

# The rvr package has the motor server, which listens to messages from here
from rvr.srv import *


# Getting robot parameters
rospy.loginfo('Getting parameters for robot.')
# min speed of the motors (i.e. 0-255 for adafruit motor shield).
minMotorSpeed = rospy.get_param('/rvr/minMotorSpeed')
rospy.loginfo('Using minMotorSpeed %s.', minMotorSpeed)
# max speed of the motors (i.e. 0-255 for adafruit motor shield).
maxMotorSpeed = rospy.get_param('/rvr/maxMotorSpeed')
rospy.loginfo('Using maxMotorSpeed %s.', maxMotorSpeed)


# node init
rospy.init_node('keyboard_listener', anonymous=False)


# Service 'motor' from motor_server.py ready?
rospy.loginfo("Waiting for service 'motor'")
rospy.wait_for_service('motor')

#  this will execute the "drive" command
def drive(direction, speed):
    # Service 'motor' from motor_server.py ready?
    rospy.wait_for_service('motor')

    # Send driving direction to motor
    try:
        # Create the handle 'motor_switcher' with the service type 'Motor'.
        # The latter automatically generates the MotorRequest and MotorResponse objects.
        motor_switcher = rospy.ServiceProxy('motor', Motor)

         # the handle can be called like a normal function
        rospy.loginfo("Switching motors to %s @ speed %s.", direction, speed)
        response = motor_switcher(direction, speed)

        # show result
        rospy.loginfo(rospy.get_caller_id() + ' says result is %s.', response.result)

    except rospy.ServiceException as e:
        rospy.logerr("Service call for 'motor' failed: %s", e)


def callback(data):
    # print out received message from the teleop_twist_keyboard
    # rospy.loginfo(rospy.get_caller_id() + ' received x=%s', data.linear.x)
    # rospy.loginfo(rospy.get_caller_id() + ' received z=%s', data.angular.z)

    # map joystick value to motor speed (i.e. 0.7 to 255)
    # @todo: check how to read this value from the launch file
    #scaleLinear = rospy.get_param('/joy_node/scaleLinear')
    scaleLinear = 0.8
    factor = scaleLinear/maxMotorSpeed

    # which command was received/key was pressed?
    if  (data.linear.x > 0.0) and (data.angular.z == 0.0):  # @todo: implement curve travel with the help of angular.z
      speed = int(data.linear.x/factor)
      rospy.loginfo("FORWARD.")
      drive("FORWARD", speed)
    # , key
    elif  (data.linear.x < 0.0) and (data.angular.z == 0.0):
      speed = int(data.linear.x/factor) * -1
      rospy.loginfo("BACKWARD.")
      drive("BACKWARD", speed)
    # j key
    elif  (data.linear.x == 0.0) and (data.angular.z > 0.0):
      speed = int(data.angular.z/factor)
      rospy.loginfo("LEFT .")
      drive("LEFT", speed)
    # l key
    elif  (data.linear.x == 0.0) and (data.angular.z < 0.0):
      speed = int(data.angular.z/factor) * -1
      rospy.loginfo("BACKWARD.")
      drive("RIGHT", speed)
    # k key
    elif  (data.linear.x == 0.0) and (data.angular.z == 0.0):
      rospy.loginfo("STOP.")
      drive("STOP", 0)


def listener():
    # subscribe the message cmd_vel
    rospy.Subscriber('cmd_vel', Twist, callback)

    # Ready
    rospy.loginfo("Ready. Control me via navigation stack/joystick/keyboard now (cmd_vel).")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
