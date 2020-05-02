#!/usr/bin/python3
# coding=utf-8
# -*- coding: utf-8 -*-

"""
This code reads the RVR battery voltage and shows the values on an OLED
via SSD1306 with a nice graphical battery level symbol. If
the battery level is lower than 50%,  ... [tbd].

It also shows the hostname, the IP and the time and CPU temoerature.

It also checks a pushbotton state, connected to BCM 17 on Raspberry Pi 3
via 10k pull-down resistor. If pushed, it calls the "shutdown now" command.
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


#-------------------
# RVR stuff
#-------------------
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), './lib/')))

from sphero_sdk import SpheroRvrObserver
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups
from sphero_sdk import DriveFlagsBitmask
from sphero_sdk import BatteryVoltageStatesEnum as VoltageStates


# create the RVR object.
# This also lets the robot do a firmware check every now and then.
print("Starting RVR observer...")
rvr = SpheroRvrObserver()

# for battery empty alarm
batteryEmptyLevel = 25  # below 25% means it is empty
batteryPercent = 0
batteryState = 0


# RVR battery voltage handler
def battery_percentage_handler(battery_percentage):
    #print("The battery has {0:2d} % left.".format(battery_percentage["percentage"]))
    # store globally
    global batteryPercent
    batteryPercent = battery_percentage["percentage"]


# RVR battery state handler
def battery_voltage_state_change_handler(battery_voltage_state):
    # Voltage states:  [unknown: 0, ok: 1, low: 2, critical: 3]
    #print("The battery voltage state is {0:1d}.".format(battery_voltage_state["state"]))
    global batteryState
    batteryState = battery_voltage_state["state"]


# wait time in seconds between different display information
waitTime = 2
# wait time for a piezo beep
waitTimePiezo = 0.2


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



# OLED resolution
WIDTH = 128
HEIGHT = 32  # Change to 64 if needed
BORDER = 1

# Use I2C for OLED
print("Starting OLED...")
# Define the OLED reset Pin
oled_reset = digitalio.DigitalInOut(board.D4)
i2c = board.I2C()
oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c, addr=0x3C, reset=oled_reset)


# --------------------
# for signal handling
# --------------------
import signal
import sys

# my signal handler
# handling program termination, i.e. CTRL C pressed
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
signal.signal(signal.SIGINT,  sig_handler)
signal.signal(signal.SIGHUP,  sig_handler)
signal.signal(signal.SIGTERM, sig_handler)


# ----------------------
# GPIO/pushbotton stuff
# ----------------------
import RPi.GPIO as GPIO
# for poweroff
from subprocess import call

# init
print('GPIO setup...')
GPIO.setmode(GPIO.BCM) # use the GPIO names, _not_ the pin numbers on the board

# Raspberry Pi pin configuration:
# pins	    BCM   BOARD
switchPin  = 17 # pin 11 (pushbutton)
ledPin     = 27 # pin 13
piezoPin   = 13 # pin 33


# setup
GPIO.setup(switchPin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # waits for LOW
GPIO.setup(ledPin,    GPIO.OUT)
GPIO.setup(piezoPin,  GPIO.OUT)

# LED OFF (low active!)
GPIO.output(ledPin, GPIO.HIGH)

# checker
buttonPressed = False


# piezo beep function
def beep(numberBeeps):
    for x in range(0, numberBeeps):
        # Piezo OFF
        GPIO.output(piezoPin, GPIO.HIGH)
        # wait
        sleep(waitTimePiezo)

        # Piezo ON (low active!)
        GPIO.output(piezoPin, GPIO.LOW)
        # "wait" (generate a square wave for the piezo)
        sleep(waitTimePiezo)


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
print('registering event handler...')
GPIO.add_event_detect(switchPin, GPIO.FALLING, callback=pushbutton_callback, bouncetime=200)


# ----------------------
# OLED stuff
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

# The fonts and sizes
size = 15
symbolWidth = 28
# text
fontText = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)
# https://fontawesome.io
# install via sudo 'sudo apt install fonts-font-awesome'
fontSymbol = ImageFont.truetype('/usr/share/fonts/truetype/font-awesome/fontawesome-webfont.ttf', size)


# ----------------------
# Voltage stuff
# ----------------------
# the battery OLED symbols
batteryEmptySymbol    = chr(0xf244) # battery-empty
# Battery states [unknown: 0, ok: 1, low: 2, critical: 3]
batteryUnknownSymbol  = chr(0xf071) # exclamation-triangle
batteryOkSymbol       = chr(0xf058) # check-circle
batteryLowSymbol      = chr(0xf0e7) # bolt
batteryCriticalSymbol = chr(0xf059) # question-triangle

# battery level (white rectangle in empty battery symbol
maxRectLength = 16


# ----------------------
# network stuff
# ----------------------
# for getting the hostname and IP of the underlying system
import socket

# the network symbol
networkSymbol = chr(0xf1eb) # fa-wifi


# -------------------------------
# CPU temperature and time stuff
# -------------------------------
import os

# the time symbol
timeSymbol = chr(0xf017) # fa-clock-o
# the temperature symbol
tempSymbol = chr(0xf21e) # fa-heartbeat  0xf2db
# the joystick symbol
joySymbol = chr(0xf11b) # fa-gamepad

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
    draw.text((0, size), ip, font=fontText, fill=255)

    # Display image.
    oled.image(image)
    oled.show()

    # wait some seconds and/or beep
    #if batteryPercent < batteryEmptyLevel:
    #    # print('BATTERY is EMPTY.')
    #    # beep n times
    #    beep(5)
    #else:
    #    sleep(waitTime)


    # --------------------------
    # Battery display
    # -------------------------

    # get RVRs battery voltage and state
    rvr.get_battery_percentage(handler=battery_percentage_handler)
    sleep(1)
    rvr.get_battery_voltage_state(handler=battery_voltage_state_change_handler)
    sleep(1)
    #rvr.enable_battery_voltage_state_change_notify(is_enabled=True)

    # clear OLED
    # Draw a black filled box to clear the image.
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
    string = "{0:2d} %".format(batteryPercent)
    draw.text((symbolWidth, 0), string, font=fontText, fill=255)
    # line 2
    if batteryState == 0:
        draw.text((0, 0), batteryUnknownSymbol, font=fontSymbol, fill=255)
    elif batteryState == 1:
        draw.text((0, 0), batteryOkSymbol, font=fontSymbol, fill=255)
    elif batteryState == 2:
        draw.text((0, 0), batteryLowSymbol, font=fontSymbol, fill=255)
    elif batteryState == 3:
        draw.text((0, 0), batteryCriticalSymbol, font=fontSymbol, fill=255)
    #draw.text((0, size), str("State: %1d" % batteryState), font=fontText, fill=255)

    # Display image.
    oled.image(image)
    oled.show()

    # wait some seconds and/or beep
    if batteryPercent < batteryEmptyLevel:
        # print('BATTERY is EMPTY.')
        # beep n times
        beep(5)
    else:
        sleep(waitTime)


    # ---------------------------------------
    # Time and CPU temp and joystick display
    # ---------------------------------------

    # clear OLED
    # Draw a black filled box to clear the image.
    draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)

    # get time
    timeString = strftime("%H:%M", localtime(time()) )

    # line 1, joystick symbol if connected or clock symbol
    if os.path.exists("/dev/input/js0"):
        draw.text((0, 0), joySymbol, font=fontSymbol, fill=255)
    else:
        draw.text((0, 0), timeSymbol, font=fontSymbol, fill=255)

    # line 1, text after symbol
    draw.text((symbolWidth, 0), timeString, font=fontText, fill=255)

    # line 2, temp symbol
    draw.text((0, size), tempSymbol, font=fontSymbol, fill=255)
    # line 2, text after symbol
    draw.text((symbolWidth, size), str(round(getCpuTemperature(), 1)) + " " + u'\N{DEGREE SIGN}'  + "C", font=fontText, fill=255)

    # Display image.
    oled.image(image)
    oled.show()

    # wait some seconds and/or beep
    if batteryPercent < batteryEmptyLevel:
        # print('BATTERY is EMPTY.')
        # beep n times
        beep(5)
    else:
        sleep(waitTime)


# wtf?
print('This line should never be reached')
