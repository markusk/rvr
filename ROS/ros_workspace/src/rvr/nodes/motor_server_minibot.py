#!/usr/bin/env python
# coding=utf-8


# name of the package(!).srv
from rvr.srv import *
import rospy

# Service nodes have to be initialised
rospy.init_node('motor_server', anonymous=False)


# for getting the hostname of the underlying system
import socket
# showing hostname
hostname = socket.gethostname()
if hostname == 'rvrmate':
    rospy.loginfo("Running on host %s.", hostname)
else:
    rospy.logwarn("Running on host %s!", hostname)


###############################
###### motor stuff
###############################
# run some parts only on the real robot
if hostname == 'rvrmate':
    rospy.loginfo("Setting up I2C...")


    # recommended for auto-disabling motors on shutdown!
    def turnOffMotors():
        rospy.loginfo("Motors turned off.")

    # turning off motors NOW - you never know...
    turnOffMotors();

else:
    rospy.logwarn("Skipping I2C setup. This is not the robot.")


# define a clean node exit
def my_exit():
  rospy.loginfo("Shutting down motor service...")
  # run some parts only on the real robot
  if hostname == 'rvrmate':
      turnOffMotors();
  rospy.loginfo("...shutting down motor service complete.")

# call this method on node exit
rospy.on_shutdown(my_exit)


# handle_motor is called with instances of MotorRequest and returns instances of MotorResponse
# The request name comes directly from the .srv filename
def handle_motor(req):
    """ In this function all the work is done :) """

    # switch xxx to HIGH, if '1' was sent
    if (req.direction == "FORWARD"): # and speed. returns result.
        # drive
        rospy.loginfo("Driving %s @ speed %s.", req.direction, req.speed)
    elif (req.direction == "BACKWARD"):
        rospy.loginfo("Driving %s @ speed %s.", req.direction, req.speed)
    elif (req.direction == "LEFT"):
        rospy.loginfo("Turning %s @ speed %s.", req.direction, req.speed)
    elif (req.direction == "RIGHT"):
        rospy.loginfo("Turning %s @ speed %s.", req.direction, req.speed)
    elif (req.direction == "STOP"):
        rospy.loginfo("Stopping.")
    else:
      rospy.logerr("Direction '%s' not implemented.", req.direction)

    # The name of the 'xyzResponse' comes directly from the Xyz.srv filename!
    # We return the speed as "okay"
    return MotorResponse(req.speed)


def motor_server():
    # This declares a new service named 'motor with the Motor service type.
    # All requests are passed to the 'handle_motor' function.
    # 'handle_motor' is called with instances of MotorRequest and returns instances of MotorResponse
    s = rospy.Service('motor', Motor, handle_motor)
    rospy.loginfo("Ready to switch motors.")

    # Keep our code from exiting until this service node is shutdown
    rospy.spin()


if __name__ == "__main__":
    motor_server()
