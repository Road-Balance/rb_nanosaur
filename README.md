# Nanosaur by Road Balance
| 🦕 The smallest NVIDIA Jetson dinosaur robot 🦖 - Reborn by Road Balance

<p align="center">
    <img src="./images/real_img.jpg" width="400" />
</p>

---

```
pip3 install wheel setuptools Adafruit_MotorHat
pip3 install -U wstool
```

## oled test

```
git clone https://github.com/JetsonHacksNano/installPiOLED
cd installPiOLED
./installPiOLED

cd pioled
sudo python3 stats.py

27 28 
$ sudo i2cdetect -y -r 0
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- 3c -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --

disp = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=0, gpio=1, i2c_address=0x3C)
```


## motor test

```
$ sudo i2cdetect -y -r 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: 60 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: 70 -- -- -- -- -- -- --

sudo python3 motor_test.py
```

## RPI Cam

```
free -m

# Disable ZRAM:
sudo systemctl disable nvzramconfig

# https://youtu.be/uvU8AXY1170?t=732

# Create 4GB swap file
sudo fallocate -l 4G /mnt/4GB.swap
sudo chmod 600 /mnt/4GB.swap
sudo mkswap /mnt/4GB.swap

# Append the following line to /etc/fstab
sudo vi /etc/fstab
sudo echo "/mnt/4GB.swap swap swap defaults 0 0" >> /etc/fstab

# REBOOT!

free -m
              total        used        free      shared  buff/cache   available
Mem:           3956         865        2551          29         539        2907
Swap:          4095           0        4095
```

# Creator

* Design by Yoon Yohan
* Progamming by Kimsooyoung
