#!/bin/bash

# from http://wiki.ros.org/indigo/Installation/UbuntuARM
sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update

echo "installing chrony; if prompted about configuration files, use the currently existing version"
sudo apt-get install -y chrony
sleep 1
sudo apt-get install -y avahi-daemon
sudo update-rc.d avahi-daemon defaults

sudo cp /usr/share/glib-2.0/schemas/gschema.dtd /usr/share/glib-2.0/schemas/gschema.dtd.bak
sudo apt-get -o Dpkg::Options::="--force-overwrite" install libglib2.0-dev

sudo dpkg --remove gdk-pixbuf libgdk-pixbuf2.0-commoninstall libgdk-pixbuf2.0-0:armhf

sudo apt-get install libgtk2.0-dev \
libxcb1=1.10-2ubuntu1              \
libxdamage1=1:1.1.4-1ubuntu1       \
libxfixes3=1:5.0.1-1ubuntu1        \
libxcursor1=1:1.1.14-1             \
libxcb-shm0=1.10-2ubuntu1          \
libxcb-render0=1.10-2ubuntu1       \
libpixman-1-0=0.30.2-2ubuntu1      \
libxrender1=1:0.9.8-1              \
libfreetype6=2.5.2-1ubuntu2        \
libfontconfig1=2.11.0-0ubuntu4     \
fontconfig-config=2.11.0-0ubuntu4     \
libxft2=2.3.1-2                    \
libxcomposite1=1:0.4.4-1

sudo apt-get -f install

sudo apt-get install -y ros-indigo-ros-base python-rosdep \
    ros-indigo-cv-bridge ros-indigo-tf2-ros ros-indigo-tf2 \
    ros-indigo-diagnostic-updater ros-indigo-angles \
    ros-indigo-eigen-conversions ros-indigo-orocos-kdl \
    ros-indigo-urdf ros-indigo-tf libxslt1-dev ros-indigo-image-transport \
    ros-indigo-ros-comm ros-indigo-rosbash ros-indigo-dynamic-reconfigure \
    ros-indigo-camera-info-manager ros-indigo-camera-calibration \
    ros-indigo-robot-state-publisher ros-indigo-tf2-geometry-msgs
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

sudo apt-get install -y python-rosinstall python-catkin-tools python-wstool python-rosinstall-generator python-pip

sudo chown -R linaro:linaro /home/linaro
chmod 600 ~/.ssh/config
