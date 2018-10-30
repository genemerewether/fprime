#!/bin/bash

# proot -0 runs as superuser

sed -i s/Linaro/Ubuntu/g /etc/lsb-release

# for ROS arm
export LANGUAGE=en_US.UTF-8
export LANG=en_US.UTF-8
locale-gen en_US.UTF-8
dpkg-reconfigure locales

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

apt-get update

apt-get install -y ros-indigo-ros-base ros-indigo-image-transport git software-properties-common libeigen3-dev symlinks python-pip \
    libopencv-dev

pip install catkin-tools
