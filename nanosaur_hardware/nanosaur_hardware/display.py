# Copyright (C) 2021, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Display
# https://github.com/adafruit/Adafruit_Python_SSD1306
import Adafruit_SSD1306

import atexit
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess

class Display:

    def __init__(self, node, timer_period=1, i2c_address=0x3C) -> None:
        """
        Reference:
        - https://github.com/adafruit/Adafruit_Python_SSD1306/blob/master/examples/shapes.py
        """
        self.node = node
        self.i2c_address = i2c_address
        # 128x32 display with hardware I2C:
        # setting gpio to 1 is hack to avoid platform detection
        self.disp = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=0, gpio=1, i2c_address=i2c_address)
        # Init display
        self.disp.begin()
        # Clear display.
        self.disp.clear()
        self.disp.display()
        # Create blank image for drawing.
        # Make sure to create image with mode '1' for 1-bit color.
        self.width = self.disp.width
        self.height = self.disp.height
        self.image = Image.new('1', (self.width, self.height))
        # Get drawing object to draw on image.
        self.draw = ImageDraw.Draw(self.image)
        # Load default font.
        self.font = ImageFont.load_default()
        # Draw some shapes.
        # First define some constants to allow easy resizing of shapes.
        padding = -2
        self.top = padding
        self.bottom = self.height - padding
        # Move left to right keeping track of the current x position for drawing shapes.
        self.x = 0
        # Init display timer
        self.timer = self.node.create_timer(timer_period, self.display_callback)
        # Configure all motors to stop at program exit
        atexit.register(self._close)

    def get_network_interface_state(self, interface):
        return subprocess.check_output('cat /sys/class/net/%s/operstate' % interface, shell=True).decode('ascii')[:-1]

    def get_ip_address(self, interface):
        if self.get_network_interface_state(interface) == 'down':
            return None
        cmd = "ifconfig %s | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'" % interface
        return subprocess.check_output(cmd, shell=True).decode('ascii')[:-1]

    # Return a string representing the percentage of CPU in use
    def get_cpu_usage(self, ):
        # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
        cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
        CPU = subprocess.check_output(cmd, shell=True)
        return CPU

    # Return a float representing the percentage of GPU in use.
    # On the Jetson Nano, the GPU is GPU0
    def get_gpu_usage(self, ):
        GPU = 0.0
        with open("/sys/devices/gpu.0/load", encoding="utf-8") as gpu_file:
            GPU = gpu_file.readline()
            GPU = int(GPU)/10
        return GPU  

    def display_callback(self):
        # Draw a black filled box to clear the image.
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)

        # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
        cmd = "free -m | awk 'NR==2{printf \"Mem:  %.0f%% %s/%s M\", $3*100/$2, $3,$2 }'"
        MemUsage = subprocess.check_output(cmd, shell=True)
        cmd = 'df -h | awk \'$NF=="/"{printf "Disk: %d/%dGB %s", $3,$2,$5}\''
        Disk = subprocess.check_output(cmd, shell=True)

        # Print the IP address
        # Two examples here, wired and wireless
        self.draw.text((self.x, self.top), "eth0: " + str(self.get_ip_address("eth0")), font=self.font, fill=255)
        # draw.text((x, top+8),     "wlan0: " + str(get_ip_address('wlan0')), font=font, fill=255)

        # Alternate solution: Draw the GPU usage as text
        # draw.text((x, top+8),     "GPU:  " +"{:3.1f}".format(GPU)+" %", font=font, fill=255)
        # We draw the GPU usage as a bar graph
        string_width, string_height = self.font.getsize("GPU:  ")
        # Figure out the width of the bar
        full_bar_width = self.width - (self.x + string_width) - 1
        gpu_usage = self.get_gpu_usage()
        # Avoid divide by zero ...
        if gpu_usage == 0.0:
            gpu_usage = 0.001
        
        draw_bar_width = int(full_bar_width * (gpu_usage / 100))
        self.draw.text((self.x, self.top + 8), "GPU:  ", font=self.font, fill=255)
        self.draw.rectangle(
            (self.x + string_width, self.top + 12, self.x + string_width + draw_bar_width, self.top + 14),
            outline=1,
            fill=1,
        )

        # Show the memory Usage
        self.draw.text((self.x, self.top + 16), str(MemUsage.decode("utf-8")), font=self.font, fill=255)
        # Show the amount of disk being used
        self.draw.text((self.x, self.top + 25), str(Disk.decode("utf-8")), font=self.font, fill=255)

        # Display image.
        # Set the SSD1306 image to the PIL image we have made, then dispaly
        self.disp.image(self.image)
        self.disp.display()  
    
    def _close(self):
        # Draw a black filled box to clear the image.
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)
        self.disp.image(self.image)
        self.disp.display()
# EOF
