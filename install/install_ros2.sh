#!/bin/bash -e

# Reference sites
# https://www.stereolabs.com/blog/ros-and-nvidia-jetson-nano/

sudo apt-get update
sudo apt-get upgrade

sudo apt-get install curl

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros2-latest.list ]; then
    sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
fi

# sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

echo "[Update the package]"
sudo apt-get update

echo "[Installing ROS and ROS Packages]"
sudo apt-get install -y ros-eloquent-ros-base

echo "source /opt/ros/eloquent/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install -y python3-pip
pip3 install -U argcomplete

echo "[Installing Colcon Build system]"
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://repo.ros2.org/ubuntu/main $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

echo "[Update the package Again]"
sudo apt update
sudo apt-get install -y python3-colcon-common-extensions

sudo apt-get install ros-eloquent-turtlesim
