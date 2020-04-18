import rclpy

from std_msgs.msg import String

# RVR stuff
# path to find the RVR lib from the public SDK
import os
import sys
from time import sleep
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

#import asyncio
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import SpheroRvrObserver
from sphero_sdk import BatteryVoltageStatesEnum as VoltageStates

# for LEDs:
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups


#try:
#rvr = SpheroRvrObserver()
#except:
#    print("\n++++++++++++++++++++++++++++++++++")
#    print("+++ Error opening serial port. +++")
#    print("+++  Is the RVR switched on??  +++")
#    print("++++++++++++++++++++++++++++++++++\n")


def battery_percentage_handler(battery_percentage):
    print('Battery percentage: ', battery_percentage)


def battery_voltage_handler(battery_voltage_state):
    print('Voltage state: ', battery_voltage_state)

    state_info = '[{}, {}, {}, {}]'.format(
        '{}: {}'.format(VoltageStates.unknown.name, VoltageStates.unknown.value),
        '{}: {}'.format(VoltageStates.ok.name, VoltageStates.ok.value),
        '{}: {}'.format(VoltageStates.low.name, VoltageStates.low.value),
        '{}: {}'.format(VoltageStates.critical.name, VoltageStates.critical.value)
    )
    print('Voltage states: ', state_info)


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('batteryPublisher')
    publisher = node.create_publisher(String, 'topic', 10)

    msg = String()

    # debug msg
    msg.data = 'ROS init done.'
    node.get_logger().info('Publishing: "%s"' % msg.data)
    publisher.publish(msg)

    # wake up RVR
    print("waking up RVR...")
    #rvr.wake()

    # give it time to wake up
    rclpy.sleep(2)

    # msg.data = 'Hello RVR: %d' % i
    msg.data = 'Hello RVR!'
    node.get_logger().info('Publishing: "%s"' % msg.data)
    publisher.publish(msg)

    #rvr.get_battery_percentage(handler=battery_percentage_handler)

    # Sleep for one second such that RVR has time to send data back
    rclpy.sleep(1)

    rclpy.spin(node)

    # close RVR
    #rvr.close()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()