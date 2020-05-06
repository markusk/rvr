#!/usr/bin/python3
# coding=utf-8
# -*- coding: utf-8 -*-

"""
This code reads the RVR battery voltage and shows the values on an OLED
via SSD1306 using I2C with a nice graphical battery level symbol. If
the battery level is lower than a specific percentate, the piezo beeps.

It also shows the hostname, the IP and the time and CPU temoerature
and a Gamepad symbol - if live conected.

It also checks a pushbotton state, connected to BCM 17 on Raspberry Pi 3
via 10k pull-down resistor. If pushed, it calls the "shutdown now" command.

For the latter this script has to be started as a system service.
"""


#-------------------
# RVR settings
#-------------------
# Valid speed values are 0-255
driveSpeed = 100
# Valid heading values are 0-359
driveHeading = 0
# the robot is "disarmed" at first; all Gamepad buttons are ignored, except the red one
armed = False
# for battery empty alarm
batteryEmptyLevel = 25  # below 25% means it is empty
batteryPercent = 0
batteryState = 0
# Battery voltage states: unknown=0, ok=1, low=2, critical=3
UNKNOWN  = 0
OK       = 1
LOW      = 2
CRITICAL = 3

# Gamepad/Joystick device path
gamepadPath = '/dev/input/js0'
# True, when Gamepad connected and device opened
gamepadOpend = False
# the joystick file handler
jsdev = 0


# OLED timing
# wait time in seconds between different display information
waitTimeOLED = 0.75
# OLED resolution
WIDTH = 128
HEIGHT = 32  # Change to 64 if needed
BORDER = 1

# Piezo
# wait time in seconds -> This is the square wave for the piezo
waitTimePiezo = 0.0004
# this is just a count, not a time unit
toneLength = 400 # 400
# pause between each tone in seconds
tonePause = 0.5 # 0.5

# Raspberry Pi pin configuration:
# pins	    BCM   BOARD
switchPin  = 17 # pin 11 (pushbutton)
ledPin     = 27 # pin 13
piezoPin   = 13 # pin 33


# for RVR and CPU temperature
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), './lib/')))

from sphero_sdk import SpheroRvrObserver
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups
from sphero_sdk import DriveFlagsBitmask
from sphero_sdk import BatteryVoltageStatesEnum as VoltageStates

# for signal handling
import signal
import sys

# for time and sleep
from time import time
from time import sleep
from time import strftime
from time import localtime

# for OLED
import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306

# for joystick is connected check (path/file exists)
import os.path

# for GPIO/pushbotton
import RPi.GPIO as GPIO
# for poweroff
from subprocess import call

# for getting the hostname and IP of the underlying system
import socket

# For Gamepad / joystick
import struct, array
from fcntl import ioctl


#-------------------
# RVR stuff
#-------------------
# create the RVR object.
# This also lets the robot do a firmware check every now and then.
print("Starting RVR observer...")
rvr = SpheroRvrObserver()

# RVR battery voltage handler
def battery_percentage_handler(battery_percentage):
    #print("The battery has {0:2d} % left.".format(battery_percentage["percentage"]))
    # store globally
    global batteryPercent
    batteryPercent = battery_percentage["percentage"]

# RVR battery state handler
def battery_voltage_state_change_handler(battery_voltage_state):
    #print("The battery voltage state is {0:1d}.".format(battery_voltage_state["state"]))
    global batteryState
    batteryState = battery_voltage_state["state"]


# Use I2C for OLED
print("Starting I2C...")
# Define the OLED reset Pin
oled_reset = digitalio.DigitalInOut(board.D4)
i2c = board.I2C()
print("Starting OLED...")
oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c, addr=0x3C, reset=oled_reset)

# OLED fonts and sizes
print("Loading fonts and symbols...")
fontSize = 15
symbolWidth = 28
# text
fontText = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', fontSize)
# https://fontawesome.io
# install via sudo 'sudo apt install fonts-font-awesome'
fontSymbol = ImageFont.truetype('/usr/share/fonts/truetype/font-awesome/fontawesome-webfont.ttf', fontSize)

# the battery OLED symbols
batteryEmptySymbol    = chr(0xf244) # battery-empty
# battery level (white rectangle in empty battery symbol)
maxRectLength = 16
# Battery states [unknown: 0, ok: 1, low: 2, critical: 3]
batteryUnknownSymbol  = chr(0xf071) # exclamation-triangle
batteryOkSymbol       = chr(0xf058) # check-circle
batteryLowSymbol      = chr(0xf0e7) # bolt
batteryCriticalSymbol = chr(0xf059) # question-triangle
# the Gamepad symbol
joySymbol = chr(0xf11b) # fa-gamepad
# the network symbol
networkSymbol = chr(0xf1eb) # fa-wifi
# Clock symbol
timeSymbol = chr(0xf017) # fa-clock-o
# the temperature symbol
chipSymbol = chr(0xf2db) # fas fa-microchip


# --------------------
# my signal handler
# --------------------
# handling program termination, i.e. CTRL C pressed
print("Creating signal handler...")
def sig_handler(_signo, _stack_frame):
    # LED OFF (low active!)
    GPIO.output(ledPin, GPIO.HIGH)
    ## GPIO cleanup
    GPIO.remove_event_detect(switchPin)
    GPIO.cleanup()
    # clear display
    oled.fill(0)
    oled.show()

    # close RVR
    print("Closing conection to RVR...")
    rvr.close()
    print("...done")
    print("RVR program terminated clean.")
    sys.exit(0)


# signals to be handled
print("Registering signal handler...")
signal.signal(signal.SIGINT,  sig_handler)
signal.signal(signal.SIGHUP,  sig_handler)
signal.signal(signal.SIGTERM, sig_handler)


# ----------------------
# GPIO/pushbotton stuff
# ----------------------
print('GPIO setup...')
GPIO.setmode(GPIO.BCM) # use the GPIO names, _not_ the pin numbers on the board

# setup
GPIO.setup(switchPin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # waits for LOW
GPIO.setup(ledPin,    GPIO.OUT)
GPIO.setup(piezoPin,  GPIO.OUT)

# LED OFF (low active!)
GPIO.output(ledPin, GPIO.HIGH)


# --------------
# Gamepad stuff
# --------------
# We'll store the Gamepad states here.
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


# piezo beep function
def beep(numberBeeps):
    for x in range(0, numberBeeps):
        # n LOW/HIGH signales generate a kind of square wave
        for n in range(0, toneLength):
            # Piezo OFF
            GPIO.output(piezoPin, GPIO.HIGH)
            # wait
            sleep(waitTimePiezo)
            # Piezo ON (low active!)
            GPIO.output(piezoPin, GPIO.LOW)
            # "wait" (generate a square wave for the piezo)
            sleep(waitTimePiezo)
    # "wait" (generate a square wave for the piezo)
    sleep(tonePause)


# pushbotten detection by interrupt, falling edge, with debouncing
def pushbutton_callback(answer):
    # LED ON (low active!)
    GPIO.output(ledPin, GPIO.LOW)

    print("Shutdown button on GPIO " + str(answer) + " pushed.")

    # clear display
    oled.fill(0)
    oled.show()
    # show some shutdown text on OLED

    # send message to all users
    call('wall +++ Shutting down Pi in 5 seconds +++', shell=True)

    # delay
    sleep(5)

    # power off
    call('sudo shutdown --poweroff "now"', shell=True)


# add button pressed event detector
print('Registering event handler...')
GPIO.add_event_detect(switchPin, GPIO.FALLING, callback=pushbutton_callback, bouncetime=200)


# -------------
# Gamepad part
# -------------
def openGamepad():
    # use the global vars!
    global gamepadOpend
    global axis_states
    global button_states
    global axis_names
    global button_names
    global axis_map
    global button_map
    global jsdev

    # Open the joystick device.
    print(('Connecting to Gamepad %s...' % gamepadPath))
    jsdev = open(gamepadPath, 'rb')

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

    # store the state
    gamepadOpend = True


# ----------------------
# OLED part
# ----------------------
# Clear display
oled.fill(0)
oled.show()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
image = Image.new("1", (oled.width, oled.height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a white background
draw.rectangle((0, 0, oled.width, oled.height), outline=255, fill=255)


def getCpuTemperature():
    tempFile = open("/sys/class/thermal/thermal_zone0/temp")
    cpu_temp = tempFile.read()
    tempFile.close()
    return float(cpu_temp)/1000


# let's go
print('ready.')


#-------------------------
# Wake up, RVR!
#-------------------------
print("Waking up RVR...")
rvr.wake()
# Give RVR time to wake up
sleep(2)
print("...done")


# --------------
# the main lopp
# --------------
while (1):
    # ------------------
    # Network data
    # ------------------
    # clear OLED
    # Draw a black filled box to clear the image.
    draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)

    # get hostname
    hostname = socket.gethostname()
    # get IP
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        ip = s.getsockname()[0]
    except:
        ip = '127.0.0.1'
    finally:
        s.close()

    # Write lines of text to display
    # line 1, network symbol
    draw.text((0, 0), networkSymbol, font=fontSymbol, fill=255)

    # line 1, hostname, after symbol
    draw.text((symbolWidth, 0), hostname, font=fontText, fill=255)
    # line 2, IP
    draw.text((0, fontSize), ip, font=fontText, fill=255)

    # Display image.
    oled.image(image)
    oled.show()

    # wait some seconds and/or beep
    #if batteryPercent < batteryEmptyLevel:
    #    # print('BATTERY is EMPTY.')
    #    # beep n times
    #    beep(5)
    #else:
    #    if armed == False:
    #       sleep(waitTimeOLED)


    # -----------------------------
    # Battery and Gamepad display
    # -----------------------------
    # get RVRs battery voltage and state
    rvr.get_battery_percentage(handler=battery_percentage_handler)
    sleep(1)
    rvr.get_battery_voltage_state(handler=battery_voltage_state_change_handler)
    sleep(1)
    #rvr.enable_battery_voltage_state_change_notify(is_enabled=True)

    # clear OLED
    draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)

    # length of rectangle in battery symbol
    rectLength = round(batteryPercent * maxRectLength / 100, 0)

    # Write lines of text to display
    # line 1, empty battery symbol
    draw.text((0, 0), batteryEmptySymbol, font=fontSymbol, fill=255)

    # add filling level as filled rectangle
    # empty: draw.rectangle((1, 3,  1, 11), outline=255, fill=255)
    # full:  draw.rectangle((1, 3, 16, 11), outline=255, fill=255)
    draw.rectangle((1, 3, rectLength, 11), outline=255, fill=255)

    # line 1, text after symbol
    # old: string = ("%.0f %%" % round(batteryPercent, 2))
    string = "{0:3d}%".format(batteryPercent)
    draw.text((symbolWidth, 0), string, font=fontText, fill=255)
    # battery state
    if batteryState == UNKNOWN:
        draw.text((symbolWidth+5*fontSize, 0), batteryUnknownSymbol, font=fontSymbol, fill=255)
    elif batteryState == OK:
        draw.text((symbolWidth+5*fontSize, 0), batteryOkSymbol, font=fontSymbol, fill=255)
    elif batteryState == LOW:
        draw.text((symbolWidth+5*fontSize, 0), batteryLowSymbol, font=fontSymbol, fill=255)
    elif batteryState == CRITICAL:
        draw.text((symbolWidth+5*fontSize, 0), batteryCriticalSymbol, font=fontSymbol, fill=255)

    # line 2, Gamepad symbol
    # Gamepad already opened?
    if gamepadOpend == True:
        # draw jostick symbol
        draw.text((0, fontSize), joySymbol, font=fontSymbol, fill=255)
        # draw "connection ok" at the end of the row
        draw.text((symbolWidth+5*fontSize, fontSize), batteryOkSymbol, font=fontSymbol, fill=255)
        # Display image.
        oled.image(image)
        oled.show()

        #---------------
        # read joystick
        #---------------
        evbuf = jsdev.read(8)
        if evbuf:
            time, value, type, number = struct.unpack('IhBB', evbuf)

            # initial button state
            #if type & 0x80:
            #     print("(initial)")

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
                                rvr.led_control.set_all_leds_color(color=Colors.red)
                                sleep(1)
                            else:
                                armed = False
                                print("+++disarmed+++")
                                # set all LEDs to green
                                rvr.led_control.set_all_leds_color(color=Colors.green)
                                sleep(1)
                        # RVR armed?
                        if armed:
                            if button == 'dpad_up':
                                print("FORWARD")
                                # drive
                                rvr.drive_with_heading(
                                    speed=driveSpeed,
                                    heading=driveHeading,
                                    flags=DriveFlagsBitmask.none.value
                                )
                            elif button == 'dpad_down':
                                print("BACKWARD")
                                # drive
                                rvr.drive_with_heading(
                                    speed=driveSpeed,
                                    heading=driveHeading,
                                    flags=DriveFlagsBitmask.drive_reverse.value
                                )
                            elif button == 'dpad_left':
                                print("LEFT")
                                # drive
                                rvr.drive_with_heading(
                                    speed   = driveSpeed,
                                    heading = 270,
                                    flags=DriveFlagsBitmask.none.value
                                )
                            elif button == 'dpad_right':
                                print("RIGHT")
                                # drive
                                rvr.drive_with_heading(
                                    speed   = driveSpeed,
                                    heading = 90,
                                    flags=DriveFlagsBitmask.none.value
                                )
                            else:
                                print(("%s pressed" % (button)))
                    else:
                        # button released
                        # RVR armed?
                        if armed:
                            if button == 'dpad_up' or button == 'dpad_down' or button == 'dpad_left' or button == 'dpad_right':
                                print("STOP")
                                # drive
                                rvr.drive_with_heading(
                                    speed=0,
                                    heading=0,
                                    flags=DriveFlagsBitmask.none.value
                                )
                            #print(("%s released" % (button)))

            # axis moved
            #if type & 0x02:
            #    axis = axis_map[number]
            #    if axis:
            #        fvalue = value / 32767.0
            #        axis_states[axis] = fvalue
            #        print(("%s: %.3f" % (axis, fvalue)))

    else:
        # Gamepad connected?
        if os.path.exists(gamepadPath):
            # draw jostick symbol
            draw.text((0, fontSize), joySymbol, font=fontSymbol, fill=255)
            # Open Gamepad
            openGamepad()
            # Display image.
            oled.image(image)
            oled.show()


    # wait some seconds and/or beep
    #if batteryPercent < batteryEmptyLevel:
    #    # print('BATTERY is EMPTY.')
    #    # beep n times
    #    beep(5)
    #else:
    if armed == False:
        sleep(waitTimeOLED)


    # --------------------------
    # Time and CPU temp display
    # --------------------------
    # clear OLED
    draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)

    # get time
    timeString = strftime("%H:%M", localtime(time()) )

    # line 1: clock symbol
    draw.text((0, 0), timeSymbol, font=fontSymbol, fill=255)
    # line 1, text after symbol
    draw.text((symbolWidth, 0), timeString, font=fontText, fill=255)

    # line 2, temp symbol
    draw.text((0, fontSize), chipSymbol, font=fontSymbol, fill=255)
    # line 2, text after symbol
    draw.text((symbolWidth, fontSize), str(round(getCpuTemperature(), 1)) + " " + u'\N{DEGREE SIGN}'  + "C", font=fontText, fill=255)

    # Display image.
    oled.image(image)
    oled.show()

    # wait some seconds and/or beep
    #if batteryPercent < batteryEmptyLevel:
    #    # print('BATTERY is EMPTY.')
    #    # beep n times
    #    beep(5)
    #else:
    if armed == False:
        sleep(waitTimeOLED)
