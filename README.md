# Nanosaur by Road Balance
| 🦕 The smallest NVIDIA Jetson dinosaur robot 🦖 - Original project from ![Raffaello Bonghi](https://nanosaur.ai/), Reborn by Road Balance

<p align="center">
    <img src="./images/nanosaur.gif" width="400" />
</p>

---

## System information

### Jetson Nano 2GB

* LT4 with Ubuntu 18.04 
* ROS2 Eloquent
* CUDA & TensorRT (ongoing)

## Brief Packages Explanation

```
├── STL => Contains all 3D printing Parts
├── nanosuar_camera => Handling Image data For RPI Camera  
├── nanosaur_hardware
│  └─ nanosaur_hardware
│     ├─ display.py => OLED Display Control
│     ├─ motor.py => Pololu DC Motor Control
│     └─ nanosaur.py => ROS2 Node for Cmd_vel Subscriber 
├── Images
├── LICENSE
├── README.md
└── STL
```

## Installation

* Quick install with Shell Scripts

```
# 1. Install Parts Dependencies
$ cd rb_nanosaur\install
$ sudo ./install_parts.sh

# 2. Install ROS2
$ sudo ./install_ros2.sh

# Then, Setup Colcon workspace
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ cd ../
$ colcon build

# 3. Install Dependencies for Camera GPU Acceleration
$ sudo ./install_ai_depend.sh
```

## Check connection for All Parts 
### oled test

```
# OLED shold be connected with 27 28 pin

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

# Try test
cd tests
sudo python3 stats.py
```


## motor test

```
# Motor Driver shold be connected with 3 5 pin

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

# Try test
cd tests
sudo python3 motor_test.py
```

## RPI Cam

```
# RPI Cam Must be connected before OS Booting

# Try test
cd tests
sudo python3 camera_test.py
```

* Grab Swap Area

| for more detail, watch [this video](https://youtu.be/uvU8AXY1170?t=732) 
```
free -m

# Disable ZRAM:
sudo systemctl disable nvzramconfig

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


# Run

```
sudo apt-get install ros-eloquent-teleop-twist-keyboard

# Ternimal 1
colcon build --symlink-install --packages-select nanosaur_hardware
. install\setup.bash
ros2 run nanosaur_hardware nanosaur

# Ternimal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


```
pip3 install wheel setuptools Adafruit_MotorHat
pip3 install -U wstool

cd /opt && \
    git clone https://github.com/dusty-nv/jetson-utils.git && \
    mkdir -p jetson-utils/build && cd jetson-utils/build && \
    cmake ../ && \
    make -j$(nproc) && \
    make install && \
    ldconfig
```

---

My Notepad (Ignore belows)

```
colcon build
. install/setup.bash
ros2 run nanosaur_hardware nanosaur

sudo apt-get install ros-eloquent-teleop-twist-keyboard

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"


```


# Creator

* Design by Yoon Yohan
* Progamming by Kimsooyoung
