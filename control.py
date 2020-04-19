#!/usr/bin/env python3
# coding=utf-8


""" Control the Sphero RVR with a Gamepad (Microsoft XBox Controller) """


#-------------------
#Joystick stuff
#-------------------
import os, struct, array
from fcntl import ioctl

# Iterate over the joystick devices.
print('Available devices:')

for fn in os.listdir('/dev/input'):
    if fn.startswith('js'):
        print(('  /dev/input/%s' % (fn)))

# We'll store the states here.
axis_states = {}
button_states = {}

# These constants were borrowed from linux/input.h
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'trottle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}

axis_map = []
button_map = []

# Open the joystick device.
fn = '/dev/input/js0'
print(('Opening %s...' % fn))
jsdev = open(fn, 'rb')

# Get the device name.
#buf = bytearray(63)
buf = array.array('B', [0] * 64)
ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
js_name = buf.tostring().rstrip(b'\x00').decode('utf-8')
print(('Device name: %s' % js_name))

# Get number of axes and buttons.
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
num_axes = buf[0]

buf = array.array('B', [0])
ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
num_buttons = buf[0]

# Get the axis map.
buf = array.array('B', [0] * 0x40)
ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

for axis in buf[:num_axes]:
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)
    axis_states[axis_name] = 0.0

# Get the button map.
buf = array.array('H', [0] * 200)
ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

for btn in buf[:num_buttons]:
    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    button_map.append(btn_name)
    button_states[btn_name] = 0

#print(('%d axes found: %s' % (num_axes, ', '.join(axis_map))))
#print(('%d buttons found: %s' % (num_buttons, ', '.join(button_map))))


#-------------------
# signal handling
#-------------------
import signal
import sys

# my signal handler
def sig_handler(_signo, _stack_frame):
    # turn off RVR LEDs
    try:
        rvr.set_all_leds(
            led_group=RvrLedGroups.all_lights.value,
            led_brightness_values=[color for _ in range(10) for color in [0, 0, 0]]
        )
        # Delay to show LEDs change
        time.sleep(1)
    finally:
        rvr.close()

    print("\n\ncontrol.py terminated clean.\n")
    sys.exit(0)


# signals to be handled
signal.signal(signal.SIGINT,  sig_handler)
signal.signal(signal.SIGHUP,  sig_handler)
signal.signal(signal.SIGTERM, sig_handler)


#-------------------
# RVR stuff
#-------------------
import os
import sys
import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), './lib/')))

from sphero_sdk import SpheroRvrObserver
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups
from sphero_sdk import DriveFlagsBitmask

# the robot is "disarmed"; all buttons are ignored, except the red one
armed = False


# create the RVR object.
# This also lets the robot do a firmware check every now and then.
rvr = SpheroRvrObserver()


#-------------------------
# Wake up, Mr. Freeman!
#-------------------------
print("Waking up RVR...")
rvr.wake()
# Give RVR time to wake up
time.sleep(2)
print("...done")

# All LEDs to green
rvr.set_all_leds(
    led_group=RvrLedGroups.all_lights.value,
    led_brightness_values=[color for x in range(10) for color in [0, 255, 0]]
)
# Delay to show LEDs change
time.sleep(1)


"""# Main event loop
try:
    rvr.wake()
    # Give RVR time to wake up
    time.sleep(2)

    rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for _ in range(10) for color in Colors.off.value]
    )

    # Delay to show LEDs change
    time.sleep(1)

    rvr.set_all_leds(
        led_group=RvrLedGroups.all_lights.value,
        led_brightness_values=[color for _ in range(10) for color in [255, 0, 0]]
    )

    # Delay to show LEDs change
    time.sleep(1)

except KeyboardInterrupt:
    print('\nProgram terminated with keyboard interrupt.')

finally:
    rvr.close() """


# joystick loop
while True:
    evbuf = jsdev.read(8)
    if evbuf:
        time, value, type, number = struct.unpack('IhBB', evbuf)

        if type & 0x80:
             print("(initial)")

        # button change
        if type & 0x01:
            button = button_map[number]
            if button:
                button_states[button] = value
                # pressed
                if value:
                    # red butoon
                    if button == 'b':
                        # arm/disarm RVR
                        if armed == False:
                            armed = True
                            print("+++armed+++")

                            # set all LEDs to red
                            rvr.set_all_leds(
                                led_group=RvrLedGroups.all_lights.value,
                                led_brightness_values=[color for x in range(10) for color in [255, 0, 0]]
                            )
                            # Delay to show LEDs change
                            time.sleep(1)
                            
                        else:
                            armed = False
                            print("+++disarmed+++")

                            # set all LEDs to green
                            rvr.set_all_leds(
                                led_group=RvrLedGroups.all_lights.value,
                                led_brightness_values=[color for _ in range(10) for color in [0, 255, 0]]
                            )

                            # Delay to show LEDs change
                            time.sleep(1)

                    # RVR armed?
                    if armed:
                        if button == 'dpad_up':
                            print("FORWARD")
                        elif button == 'dpad_down':
                            print("BACKWARD")
                        elif button == 'dpad_left':
                            print("LEFT")
                        elif button == 'dpad_right':
                            print("RIGHT")
                        else:
                            print(("%s pressed" % (button)))
                else:
                    # button released
                    # RVR armed?
                    if armed:
                        if button == 'dpad_up' or button == 'dpad_down' or button == 'dpad_left' or button == 'dpad_right':
                            print("STOP")
                        #print(("%s released" % (button)))

        # axis moved
        #if type & 0x02:
        #    axis = axis_map[number]
        #    if axis:
        #        fvalue = value / 32767.0
        #        axis_states[axis] = fvalue
        #        print(("%s: %.3f" % (axis, fvalue)))
