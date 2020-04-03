#!/usr/bin/env python3
# coding=utf-8

"""
This is a service node (server) to control motors on the RVR.
It controls the motors via [... tbd ...] on a Raspberry Pi.
I expects the messsages "FORWARD, BACKWARD, LEFT, RIGHT, STOP".

Author:  Markus Knapp, 2020
Website: https://direcs.de
"""


# name of the package.srv
from rvr.srv import *
import rospy

# Service nodes have to be initialised
rospy.init_node('motor_server', anonymous=False)

# Getting robot parameters
rospy.loginfo('Getting hostname for robot from config.')
configHostname = rospy.get_param('/rvr/hostname')
rospy.loginfo("Awaiting hostname %s.", configHostname)

# for coroutines
import asyncio

# getting the hostname of the underlying operating system
import socket
# showing hostname
hostname = socket.gethostname()
if hostname == configHostname:
    rospy.loginfo("Running on host %s.", hostname)
else:
    rospy.logwarn("Running on host %s, and not %s as set in main launchfile!" % (hostname, configHostname))


# RVR stuff
#if hostname == configHostname:
#    rospy.loginfo("Setting up RVR...")
#
#    import os
#    import sys
#    # path to find the RVR lib from the public SDK
#    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../lib/')))
#
#    import asyncio
#    from sphero_sdk import SpheroRvrAsync
#    from sphero_sdk import SerialAsyncDal
#    from sphero_sdk import DriveFlagsBitmask
#
#
#    loop = asyncio.get_event_loop()
#
#    rvr = SpheroRvrAsync(
#        dal=SerialAsyncDal(
#            loop
#        )
#    )
#    rospy.loginfo("...done.")
#else:
#    rospy.logwarn("Skipping RVR setup. This is not the robot.")


# define a clean ROS node exit
def my_exit():
    rospy.loginfo("Shutting down motor service...")
    # run some parts only on the real robot
    if hostname == configHostname:
        # turn Off Motors()
        rospy.loginfo("...shutting down motor service complete.")


# call this method on ROS node exit
rospy.on_shutdown(my_exit)


# handle_motor is called with instances of MotorRequest and returns instances of MotorResponse
# The ROS request name comes directly from the .srv filename
#async def handle_motor(req):
def handle_motor(req):
    """ In this function all the work is done :) """

    # switch motors
    if (req.direction == "FORWARD"): # and speed. returns result.
        # drive
        rospy.loginfo("Driving %s @ speed %s.", req.direction, req.speed)
#        if hostname == configHostname:
#            await rvr.drive_with_heading(
#                speed  = req.speed,  # Valid speed values are 0-255
#                heading=        90,  # Valid heading values are 0-359
#                flags=DriveFlagsBitmask.none.value
#            )
#            # Delay to allow RVR to drive
#            await asyncio.sleep(1)
    elif (req.direction == "STOP"):
        rospy.loginfo("Stopping.")
#        if hostname == configHostname:
#            await rvr.drive_with_heading(
#                speed  =         0,  # Valid speed values are 0-255
#                heading=         0,  # Valid heading values are 0-359
#                flags=DriveFlagsBitmask.none.value
#            )
#            # Delay to allow RVR to drive
#            await asyncio.sleep(1)
    else:
      rospy.logerr("Direction '%s' not implemented.", req.direction)

    # The name of the 'xyzResponse' comes directly from the Xyz.srv filename!
    # We return the speed as "okay"
    return MotorResponse(req.speed)


# async def motor_server():
def motor_server():
    # This declares a new service named 'motor with the Motor service type.
    # All requests are passed to the 'handle_motor' function.
    # 'handle_motor' is called with instances of MotorRequest and returns instances of MotorResponse
    s = rospy.Service('motor', Motor, handle_motor)
    rospy.loginfo("Ready to switch motors.")

    # start RVR comms
    if hostname == configHostname:
        rospy.loginfo("Waking up RVR...")
#        await rvr.wake()
        # Give RVR time to wake up (2 seconds)
#        await asyncio.sleep(2)
        # reset YAW
#        await rvr.reset_yaw()
        rospy.loginfo("...done.")

    # Keep our code from exiting until this service node is shutdown
    rospy.spin()


if __name__ == "__main__":
	motor_server()
#    try:
#        loop.run_until_complete(
#            motor_server()
#        )

"""
    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')

        if hostname == configHostname:
            loop.run_until_complete(
                # stop RVR comms
                rvr.close()
            )

    finally:
        if hostname == configHostname:
            if loop.is_running():
                # stop RVR comms
                loop.close()
"""
