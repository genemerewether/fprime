#!/bin/bash

# proot -0 runs as superuser

sed -i s/Linaro/Ubuntu/g /etc/lsb-release

# for ROS arm
export LANGUAGE=en_US.UTF-8
export LANG=en_US.UTF-8
locale-gen en_US.UTF-8
dpkg-reconfigure locales

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

apt-get update

apt-get install -y ros-indigo-ros-base ros-indigo-image-transport \
        ros-indigo-trac-ik ros-indigo-orocos-kdl \
        ros-indigo-sophus libsuitesparse-dev \
    libeigen3-dev libopencv-dev \
    git software-properties-common python-pip \
    symlinks

pip install catkin-tools

dpkg -i --ignore-depends=libz1,libgcc-s1 cross_toolchain/downloads/$(ls cross_toolchain/downloads/mv_*.deb | tail -1)

#TODO(mereweth) - edit /var/lib/dpkg/status to remove mv dep on libz1,libgcc-s1
