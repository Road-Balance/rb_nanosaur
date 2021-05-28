#! /usr/bin/python3
# Copyright (c) 2017 Adafruit Industries
# Author: Tony DiCola & James DeVito
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# Portions copyright (c) NVIDIA 2019
# Portions copyright (c) JetsonHacks 2019

import time

import Adafruit_SSD1306  # This is the driver chip for the Adafruit PiOLED

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess


def get_network_interface_state(interface):
    return subprocess.check_output(
        "cat /sys/class/net/%s/operstate" % interface, shell=True
    ).decode("ascii")[:-1]


def get_ip_address(interface):
    if get_network_interface_state(interface) == "down":
        return None
    cmd = (
        "ifconfig %s | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'"
        % interface
    )
    return subprocess.check_output(cmd, shell=True).decode("ascii")[:-1]


# Return a string representing the percentage of CPU in use


def get_cpu_usage():
    # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
    cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
    CPU = subprocess.check_output(cmd, shell=True)
    return CPU


# Return a float representing the percentage of GPU in use.
# On the Jetson Nano, the GPU is GPU0


def get_gpu_usage():
    GPU = 0.0
    with open("/sys/devices/gpu.0/load", encoding="utf-8") as gpu_file:
        GPU = gpu_file.readline()
        GPU = int(GPU) / 10
    return GPU


# 128x32 display with hardware I2C:
# setting gpio to 1 is hack to avoid platform detection
disp = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=0, gpio=1, i2c_address=0x3C)

# Initialize library.
disp.begin()

# Clear display.
disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new("1", (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)
# Load default font.
font = ImageFont.load_default()

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = 2
top = padding
bottom = height - padding
# Move left to right keeping track of the current x position for drawing shapes.
x = 0

# Draw a black filled box to clear the image.
draw.rectangle((0, 0, width, height), outline=0, fill=0)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = -2
top = padding
bottom = height - padding
# Move left to right keeping track of the current x position for drawing shapes.
x = 0


while True:

    # Draw a black filled box to clear the image.
    draw.rectangle((0, 0, width, height), outline=0, fill=0)

    x = 0
    shape_width = 20
    padding = 2

    # Draw an ellipse.
    draw.ellipse((x, top, x + shape_width, bottom), outline=255, fill=0)
    x += shape_width + padding
    # Draw a rectangle.
    draw.rectangle((x, top, x + shape_width, bottom), outline=255, fill=0)
    x += shape_width + padding
    # Draw a triangle.
    draw.polygon(
        [(x, bottom), (x + shape_width / 2, top), (x + shape_width, bottom)],
        outline=255,
        fill=0,
    )
    x += shape_width + padding
    # Draw an X.
    draw.line((x, bottom, x + shape_width, top), fill=255)
    draw.line((x, top, x + shape_width, bottom), fill=255)
    x += shape_width + padding

    draw.text((x, top), "Hello", font=font, fill=255)
    draw.text((x, top + 20), "World!", font=font, fill=255)

    # Display image.
    # Set the SSD1306 image to the PIL image we have made, then dispaly
    disp.image(image)
    disp.display()
    # 1.0 = 1 second; The divisor is the desired updates (frames) per second
    time.sleep(1.0 / 4)
