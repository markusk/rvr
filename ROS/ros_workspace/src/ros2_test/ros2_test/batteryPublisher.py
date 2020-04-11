import rclpy

from std_msgs.msg import String

# RVR stuff
# path to find the RVR lib from the public SDK
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

import asyncio
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import BatteryVoltageStatesEnum as VoltageStates

# for LEDs:
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups


loop = asyncio.get_event_loop()

try:
    rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop))
except:
    print("\n++++++++++++++++++++++++++++++++++")
    print("+++ Error opening serial port. +++")
    print("+++  Is the RVR switched on??  +++")
    print("++++++++++++++++++++++++++++++++++\n")


async def main(args=None):
    rclpy.init(args=args)

    # create ROS node
    node = rclpy.create_node('batteryPublisher')
    publisher = node.create_publisher(String, 'topic', 10)

    msg = String()
    i = 0

    def timer_callback():
        nonlocal i
        msg.data = 'Hello RVR: %d' % i
        i += 1
        node.get_logger().info('Publishing: "%s"' % msg.data)
        publisher.publish(msg)

    timer_period = 0.5  # seconds
    timer = node.create_timer(timer_period, timer_callback)

    rclpy.spin(node)

    # Destroy the timer attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        loop.run_until_complete(
            main()
        )

    except KeyboardInterrupt:
        print("\nProgram terminated with keyboard interrupt.")

        loop.run_until_complete(
            # rvr.close()
            print("rvr close. :)")
        )

    finally:
        if loop.is_running():
            loop.close()
