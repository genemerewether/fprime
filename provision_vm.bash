#!/bin/bash

ROS2_DISTRO=bouncy

sudo apt-get install -y xauth

########## ROS1 ##########

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

source /opt/ros/kinetic/setup.bash

sudo apt-get install -y python-catkin-tools libgoogle-glog-dev automake

########## RotorS ##########

mkdir -p /quest_ws/src
cd /quest_ws
catkin init
cd src
git clone https://github.com/catkin/catkin_simple
git clone https://github.com/ethz-asl/eigen_catkin.git
git clone https://github.com/ethz-asl/eigen_checks.git
git clone https://github.com/ethz-asl/gflags_catkin.git
git clone https://github.com/ethz-asl/glog_catkin.git
git clone git@github.com:ethz-asl/mav_comm.git
git clone https://github.com/OctoMap/octomap.git
git clone https://github.com/OctoMap/octomap_msgs.git
git clone https://github.com/OctoMap/octomap_ros.git
git clone git@github.com:ethz-asl/rotors_simulator.git

########## ROS2 ##########

sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt-get update

sudo apt install -y ros-${ROS2_DISTRO}-ros-base python3-pip python3-colcon-common-extensions ros-${ROS2_DISTRO}-ros1-bridge ros-${ROS_DISTRO}-demo-nodes-cpp
sudo -H pip3 install argcomplete

#echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> ~/.bashrc