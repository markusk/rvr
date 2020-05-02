#!/usr/bin/python3
# coding=utf-8

"""
Sends n beeps to a piezo buzzer.
Usage for 5 beeps: beep.py 5
"""

# wait time in seconds -> This is the square wave for the piezo
waitTime = 0.0004
# this is just a count, not a time unit
toneLength = 400
# pause between each tone in seconds
tonePause = 0.5

# for getting arguments
import sys

# check arguments
if len(sys.argv) != 2:
#    print("1. argument: " + sys.argv[1])
#    print("2. argument: " + sys.argv[2])
    print("ERROR. Correct usage for three beeps is: " + sys.argv[0] + " 3")
    sys.exit(-1)


# for GPIO pin usage
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!")

# for sleep
import time

##
## GPIO stuff
##
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM) # use the GPIO names, _not_ the pin numbers on the board
# Raspberry Pi pin configuration:
# pins	    BCM   BOARD
piezoPin  = 13 # pin

# GPIO setup
GPIO.setup(piezoPin, GPIO.OUT)


# for signal handling
import signal
import sys

# my signal handler
def sig_handler(_signo, _stack_frame):
    # GPIO cleanup
    GPIO.cleanup()
    print("beep terminated clean.")
    sys.exit(0)

# signals to be handled
signal.signal(signal.SIGINT,  sig_handler)
signal.signal(signal.SIGHUP,  sig_handler)
signal.signal(signal.SIGTERM, sig_handler)


######
###### Beeping
######
for x in range(0, int(sys.argv[1])):
    # 10 LOW/HIGH signales generate a kind of square wave
    for n in range(0, toneLength):
        # Piezo OFF
        GPIO.output(piezoPin, GPIO.HIGH)
        # wait
        time.sleep(waitTime)

        # Piezo ON (low active!)
        GPIO.output(piezoPin, GPIO.LOW)
        # "wait" (generate a square wave for the piezo)
        time.sleep(waitTime)

    # "wait" (generate a square wave for the piezo)
    time.sleep(tonePause)

# GPIO cleanup
GPIO.cleanup()
