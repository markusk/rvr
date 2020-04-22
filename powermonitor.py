#!/usr/bin/python3
# coding=utf-8
# -*- coding: utf-8 -*-

"""
This code reads the RVR battery voltage and shows the values on an OLED
via SSD1306 with a nice graphical battery level symbol. If
the battery level is lower than 50%,  ... [tbd].

It also shows the hostname, the IP and the time and CPU temoerature.

to do:
It also checks a pushbotton state, connected to #17 (pin 11 on Raspberry Pi 3)
via 10k pull-down resistor. If pushed, it calls the "shutdown now" command.
"""


# for battery empty alarm
batteryEmptyLevel = 50  # below 50% means it is empty
batteryIsEmpty = False


# wait time in seconds between different display information
waitTime = 2
# wait time for a piezo beep
waitTimePiezo = 0.2


# for time and sleep
import time

# for OLED
import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306

# for joystick is connected check (path/file exists)
import os.path


# Define the Reset Pin
oled_reset = digitalio.DigitalInOut(board.D4)

# OLED resolution
WIDTH = 128
HEIGHT = 32  # Change to 64 if needed
BORDER = 1

# Use for I2C.
i2c = board.I2C()
oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c, addr=0x3C, reset=oled_reset)


# --------------------
# for signal handling
# --------------------
import signal
import sys

# my signal handler
def sig_handler(_signo, _stack_frame):
    # LED OFF (low active!)
    #GPIO.output(ledPin, GPIO.HIGH)
    ## GPIO cleanup
    #GPIO.remove_event_detect(switchPin)
    #GPIO.cleanup()
    # clear display
    oled.clear()
    oled.display()
    print("powermonitor terminated clean.")
    sys.exit(0)

# signals to be handled
signal.signal(signal.SIGINT,  sig_handler)
signal.signal(signal.SIGHUP,  sig_handler)
signal.signal(signal.SIGTERM, sig_handler)


# ----------------------
# GPIO/pushbotton stuff
# ----------------------
#import RPi.GPIO as GPIO
# for poweroff
#from subprocess import call

# init
#GPIO.setmode(GPIO.BCM) # use the GPIO names, _not_ the pin numbers on the board

# Raspberry Pi pin configuration:
# pins	    BCM   BOARD
#switchPin  = 17 # pin 11
#ledPin     = 18 # pin 12
#piezoPin   = 25 # pin


# setup
#print('setup...')
#GPIO.setup(switchPin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # waits for LOW
#GPIO.setup(ledPin,    GPIO.OUT)
#GPIO.setup(piezoPin,  GPIO.OUT)

# LED OFF (low active!)
#GPIO.output(ledPin, GPIO.HIGH)

# checker
#buttonPressed = False


# piezo beep function
def beep(numberBeeps):
    for x in range(0, numberBeeps):
        # Piezo OFF
        #GPIO.output(piezoPin, GPIO.HIGH)
        # wait
        time.sleep(waitTimePiezo)

        # Piezo ON (low active!)
        #GPIO.output(piezoPin, GPIO.LOW)
        # "wait" (generate a square wave for the piezo)
        time.sleep(waitTimePiezo)


# switch detection by interrupt, falling edge, with debouncing
#def my_callback(answer):
#    # LED ON (low active!)
#    GPIO.output(ledPin, GPIO.LOW)
#
#    print("Shutdown button on GPIO " + str(answer) + " pushed.")
#
#    # clear display
#    oled.clear()
#    oled.display()
#    # show some shutdown text on OLED
#
#    # send message to all users
#    call('wall +++ Shutting down Pi in 5 seconds +++', shell=True)
#
#    # delay
#    time.sleep(5)
#
#    # power off
#    call('sudo shutdown --poweroff "now"', shell=True)


# add button pressed event detector
#print('registering event handler...')
#GPIO.add_event_detect(switchPin, GPIO.FALLING, callback=my_callback, bouncetime=200)


# ----------------------
# OLED stuff
# ----------------------

# Clear display.
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

# the battery symbols
batteryEmpty = unichr(0xf244) # <25% = minVoltage

# battery level (white rectangle in empty battery symbol
maxRectLength = 16

# min and max voltages
measuredVoltage = 0.0
minVoltage      = 3*3.3 # 3S LiPo-Battery with 3 x 3.3Volt =  9.9 Volt (empty battery)
maxVoltage      = 3*4.2 # 3S LiPo-Battery with 3 x 4.2Volt = 12.6 Volt (full  battery)

# the AD converter object
#adc = Adafruit_ADS1x15.ADS1015()
# Gain 1 means, max a value of +4.096 Volt (+4,096 Volt in Europe) on the ADC channel, resulting in a 'value' of +2047.
#GAIN = 1


# ----------------------
# network stuff
# ----------------------
# for getting the hostname and IP of the underlying system
import socket
import subprocess

# the network symbol
networkSymbol = unichr(0xf1eb) # fa-wifi


# -------------------------------
# CPU temperature and time stuff
# -------------------------------
import os

# the time symbol
timeSymbol = unichr(0xf017) # fa-clock-o
# the temperature symbol
tempSymbol = unichr(0xf21e) # fa-heartbeat  0xf2db
# the joystick symbol
joySymbol = unichr(0xf11b) # fa-gamepad

def getCpuTemperature():
    tempFile = open("/sys/class/thermal/thermal_zone0/temp")
    cpu_temp = tempFile.read()
    tempFile.close()
    return float(cpu_temp)/1000



# let's go
print('ready.')


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
    # get IP via shell
    ip = subprocess.check_output(['hostname', '-I'])
    # do we have an IP?
    if len(ip) >= 7:
        # find first space and cut string at this index
        ip4string = ip[:ip.index(" ")]
    else:
        ip4string = "-"

    # Write lines of text to display
    # line 1, network symbol
    draw.text((0, 0), networkSymbol, font=fontSymbol, fill=255)

    # line 1, hostname, after symbol
    draw.text((symbolWidth, 0), hostname, font=fontText, fill=255)
    # line 2, IP
    draw.text((0, size), ip4string, font=fontText, fill=255)

    # Display image.
    oled.image(image)
    oled.show()

    # wait some seconds and/or beep
    if batteryIsEmpty is True:
        # print('BATTERY is EMPTY.')
        # beep n times
        beep(5)
    else:
        time.sleep(waitTime)


    # --------------------------
    # Battery display
    # -------------------------

    # clear OLED
    # Draw a black filled box to clear the image.
    draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)

    # read AD converter (battery voltage)
    # use channel 0 on IC
    # value = adc.read_adc(0, gain=GAIN)
    value = 175

    # 13.44 Volt battery voltage resulted in 2,94 Volt on the ADC channel with my circuit (voltage divider w/ two resistors (39k + 11k)).
    # This resulted in a ADC 'value' of 1465.
    # The conversion factor for the battery voltage is then: 1465 / 13.44 = 109.00297619047619
    #
    measuredVoltage = (value / 109.00297619047619)
    if measuredVoltage < 0:
        measuredVoltage = 0
    # print("Value: %d" % value)
    # print("Battery: %.1f Volt" % voltage)

    # percent calculation
    convertedVoltage = measuredVoltage - minVoltage
    percent = convertedVoltage / (maxVoltage-minVoltage) * 100
    if percent < 0:
        percent = 0

    # --------------------------------------
    # this is for the battery alarn / piezo
    # --------------------------------------
    if percent < batteryEmptyLevel:
        batteryIsEmpty = True
    else:
        batteryIsEmpty = False

    # rectangle in battery symbol
    rectLength = round(percent * maxRectLength / 100, 0)

    # Write lines of text to display
    # line 1, empty battery symbol
    draw.text((0, 0), batteryEmpty, font=fontSymbol, fill=255)
    # add filling level as filled rectangle

    # empty: draw.rectangle((1, 3,  1, 11), outline=255, fill=255)
    # full:  draw.rectangle((1, 3, 16, 11), outline=255, fill=255)
    draw.rectangle((1, 3, rectLength, 11), outline=255, fill=255)

    # line 1, text after symbol
    string = ("%.0f %%" % round(percent, 2))
    draw.text((symbolWidth, 0), string, font=fontText, fill=255)
    # line 2
    draw.text((0, size), str("%.2f Volt" % measuredVoltage), font=fontText, fill=255)

    # Display image.
    oled.image(image)
    oled.show()

    # wait some seconds and/or beep
    if batteryIsEmpty is True:
        # print('BATTERY is EMPTY.')
        # beep n times
        beep(5)
    else:
        time.sleep(waitTime)


    # ---------------------------------------
    # Time and CPU temp and joystick display
    # ---------------------------------------

    # clear OLED
    # Draw a black filled box to clear the image.
    draw.rectangle((0, 0, oled.width, oled.height), outline=0, fill=0)

    # get time
    timeString = time.strftime("%H:%M", time.localtime(time.time()) )

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
    if batteryIsEmpty is True:
        # print('BATTERY is EMPTY.')
        # beep n times
        beep(5)
    else:
        time.sleep(waitTime)


# wtf?
print('This line should never be reached')
