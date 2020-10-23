#!/usr/bin/python3
# coding=utf-8

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../lib/')))

import asyncio
from sphero_sdk import SpheroRvrAsync
from sphero_sdk import SerialAsyncDal
from sphero_sdk import BatteryVoltageStatesEnum as VoltageStates

# for LEDs:
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups




loop = asyncio.get_event_loop()

rvr = SpheroRvrAsync(
    dal=SerialAsyncDal(
        loop
    )
)


async def main():
    """ This program demonstrates how to retrieve the battery state of RVR.
    """

    await rvr.wake()

    # Give RVR time to wake up
    await asyncio.sleep(2)

    # turn all LEDs OFF
    #await rvr.set_all_leds(
    #    led_group=RvrLedGroups.all_lights.value,
    #    led_brightness_values=[color for _ in range(10) for color in Colors.off.value]
    #)


    # battery in %
    battery_percentage = await rvr.get_battery_percentage()
    print("Battery percentage: ", battery_percentage['percentage'], "%")

    # battery state
    battery_voltage_state = await rvr.get_battery_voltage_state()
    print('Voltage state: ', battery_voltage_state['state'])

    # battery state
    state_info = '[{}, {}, {}, {}]'.format(
        '{}: {}'.format(VoltageStates.unknown.name, VoltageStates.unknown.value),
        '{}: {}'.format(VoltageStates.ok.name, VoltageStates.ok.value),
        '{}: {}'.format(VoltageStates.low.name, VoltageStates.low.value),
        '{}: {}'.format(VoltageStates.critical.name, VoltageStates.critical.value)
    )
    print('Voltage states: ', state_info)


    # Delay to show LEDs change
    # await asyncio.sleep(1)

    # set LEDs depending on the battery state
    if battery_percentage['percentage'] > 66:
    	# All LEDs to green [okay]
    	await rvr.set_all_leds(
        	led_group=RvrLedGroups.all_lights.value,
        	led_brightness_values=[color for x in range(10) for color in [0, 255, 0]]
    	)
    elif battery_percentage['percentage'] > 33:
    	# All LEDs to orange [low]
    	await rvr.set_all_leds(
        	led_group=RvrLedGroups.all_lights.value,
        	led_brightness_values=[color for x in range(10) for color in [255, 165, 0]]
    	)
    elif battery_percentage['percentage'] < 33:
    	# All LEDs to red [critical]
    	await rvr.set_all_leds(
        	led_group=RvrLedGroups.all_lights.value,
        	led_brightness_values=[color for x in range(10) for color in [255, 0, 0]]
    	)

    await rvr.close()


if __name__ == '__main__':
    try:
        loop.run_until_complete(
            main()
        )

    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')

        # turn all LEDs OFF
        #await rvr.set_all_leds(
        #    led_group=RvrLedGroups.all_lights.value,
        #    led_brightness_values=[color for _ in range(10) for color in Colors.off.value]
        #)

        loop.run_until_complete(
            rvr.close()
        )

    finally:
        if loop.is_running():
            loop.close()
