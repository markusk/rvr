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

#try:
rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop))
#except:
#    print("\n++++++++++++++++++++++++++++++++++")
#    print("+++ Error opening serial port. +++")
#    print("+++  Is the RVR switched on??  +++")
#    print("++++++++++++++++++++++++++++++++++\n")


async def main(args=None):
    # ROS init
    rclpy.init(args=args)

    # create ROS node
    node = rclpy.create_node('batteryPublisher')
    publisher = node.create_publisher(String, 'topic', 10)

    msg = String()

    # msg.data = 'Hello RVR: %d' % i
    msg.data = 'Hello RVR!'

    node.get_logger().info('Publishing: "%s"' % msg.data)
    publisher.publish(msg)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        asyncio.ensure_future(
            main()
        )
        loop.run_forever()

    except KeyboardInterrupt:
        print("\nProgram terminated with keyboard interrupt.")

        loop.run_until_complete(
            asyncio.gather(
                # rvr.sensor_control.clear(),
                # rvr.close()
                print("rvr close. :)")
            )
        )

    finally:
        if loop.is_running():
            loop.close()
