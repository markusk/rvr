#!/usr/bin/python
# coding=utf-8

# import time

# for getting arguments
import sys

# check arguments
if len(sys.argv) != 3:
#    print "1. argument: " + sys.argv[1]
#    print "2. argument: " + sys.argv[2]
    print "ERROR. Correct Usage is: " + sys.argv[0] + " text1 text2"
    sys.exit(-1)

# check arg length
length = 15
if len(sys.argv[1]) > length:
    print "ERROR: Text 1 exceeds length of %d." % length
    if len(sys.argv[2]) > length:
        print "ERROR: Text 2 exceeds length of %d." % length
    sys.exit(-1)

if len(sys.argv[2]) > length:
    print "ERROR: Text 2 exceeds length of %d." % length
    if len(sys.argv[1]) > length:
        print "ERROR: Text 1 exceeds length of %d." % length
    sys.exit(-1)


# OLED stuff
import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306


# Define the Reset Pin
oled_reset = digitalio.DigitalInOut(board.D4)

# OLED resolution
WIDTH = 128
HEIGHT = 32  # Change to 64 if needed
BORDER = 1

# Use for I2C.
i2c = board.I2C()
oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c, addr=0x3C, reset=oled_reset)

# Clear display.
oled.fill(0)
oled.show()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
image = Image.new("1", (oled.width, oled.height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0,0,width,height), outline=0, fill=0)

# Load TTF font.
size = 15
font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)

# Send text to OLED (arguments from command line)
draw.text((0, 0),    sys.argv[1],  font=font, fill=255)
draw.text((0, size), sys.argv[2], font=font, fill=255)

# Display image.
oled.image(image)
oled.show()
