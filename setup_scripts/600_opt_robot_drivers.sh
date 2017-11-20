#!/bin/bash

mkdir ~/Documents/
cd ~/Documents/
svn checkout https://codex.cs.bham.ac.uk/svn/int-robot/course/socspioneer
echo '
# Make SoCS Pioneer visible
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Documents/socspioneer' >> ~/.bashrc

sudo apt-get install -qq ros-indigo-hokuyo3d ros-indigo-hokuyo-node \
ros-indigo-p2os-doc ros-indigo-p2os-launch ros-indigo-p2os-teleop \
ros-indigo-p2os-driver ros-indigo-p2os-msgs \
ros-indigo-joy ros-indigo-joystick-drivers \
ros-indigo-joy-listener ros-indigo-joy-teleop \
ros-indigo-joy-mouse
