#!/usr/bin/env python
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


# getting the hostname of the underlying operating system
import socket
# showing hostname
hostname = socket.gethostname()
if hostname == 'rvr':
    rospy.loginfo("Running on host %s.", hostname)
else:
    rospy.logwarn("Running on host %s!", hostname)


# RVR stuff
if hostname == 'rvr':
    rospy.loginfo("Setting up RVR...")
    
    import os
    import sys
    # path to find the RVR lib from the SDK
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), './lib/')))

    import asyncio
    from sphero_sdk import SpheroRvrAsync
    from sphero_sdk import SerialAsyncDal
    from sphero_sdk import DriveFlagsBitmask


    loop = asyncio.get_event_loop()

    rvr = SpheroRvrAsync(
        dal=SerialAsyncDal(
            loop
        )
    )
else:
    rospy.logwarn("Skipping RVR setup. This is not the robot.")


# define a clean ROS node exit
def my_exit():
    rospy.loginfo("Shutting down motor service...")
    # run some parts only on the real robot
    if hostname == 'rvr':
        # turn Off Motors()
    rospy.loginfo("...shutting down motor service complete.")


# call this method on ROS node exit
rospy.on_shutdown(my_exit)


async def main():
    await rvr.wake()

    # Give RVR time to wake up
    await asyncio.sleep(2)

    await rvr.reset_yaw()

    await rvr.drive_with_heading(
        speed=128,  # Valid speed values are 0-255
        heading=0,  # Valid heading values are 0-359
        flags=DriveFlagsBitmask.none.value
    )

    # Delay to allow RVR to drive
    await asyncio.sleep(1)

    await rvr.drive_with_heading(
        speed=128,  # Valid speed values are 0-255
        heading=0,  # Valid heading values are 0-359
        flags=DriveFlagsBitmask.drive_reverse.value
    )

    # Delay to allow RVR to drive
    await asyncio.sleep(1)

    await rvr.drive_with_heading(
        speed=128,   # Valid speed values are 0-255
        heading=90,  # Valid heading values are 0-359
        flags=DriveFlagsBitmask.none.value
    )

    # Delay to allow RVR to drive
    await asyncio.sleep(1)

    await rvr.drive_with_heading(
        speed=128,    # Valid speed values are 0-255
        heading=270,  # Valid heading values are 0-359
        flags=DriveFlagsBitmask.none.value
    )

    # Delay to allow RVR to drive
    await asyncio.sleep(1)

    await rvr.drive_with_heading(
        speed=0,    # Valid heading values are 0-359
        heading=0,  # Valid heading values are 0-359
        flags=DriveFlagsBitmask.none.value
    )

    # Delay to allow RVR to drive
    await asyncio.sleep(1)

    await rvr.close()


if __name__ == '__main__':
    try:
        loop.run_until_complete(
            main()
        )

    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')

        loop.run_until_complete(
            rvr.close()
        )

    finally:
        if loop.is_running():
            loop.close()
