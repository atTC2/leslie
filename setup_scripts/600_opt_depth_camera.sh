#!/bin/bash

trap 'exit' ERR

echo 'Installing Astra Camera'

# --- Determine the location of this script. ---
scriptName='600_opt_depth_camera.sh'
if [ ! -z "${0+x}" ]\
&& [ -e "${0}" ]\
&& [[ "${0}" == *"${scriptName}" ]]\
&& [ $(basename "${0}") = "${scriptName}" ]; then
    scriptLocation="$(dirname "${0}")"
elif [ -e "$(pwd)/${scriptName}" ]; then
    scriptLocation="$(pwd)"
elif [ -e "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/${scriptName}" ]; then
    scriptLocation="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
    error_exit 'Can not determine the location of this script...\nPlease run this script from the same folder as the script.' 1
fi

# Run setup.bash as it needs roscd
cd "${scriptLocation}/../../../devel"
source setup.bash
# Get and setup Astra Camera
roscd
cd ../src
# Currently at catkin_ws/src
git clone https://github.com/orbbec/ros_astra_camera.git
git clone https://github.com/orbbec/ros_astra_launch.git
git clone https://github.com/ros-drivers/usb_cam.git
sed -i '116s/.*/    node_.param("camera_frame_id", img_.header.frame_id, std::string("camera_rgb_optical_frame"));/' usb_cam/nodes/usb_cam_node.cpp
roscd
source setup.bash
cd ..
# Currently at catkin_ws
sudo apt-get install -qq ros-indigo-cv-bridge libudev-dev ros-indigo-rgbd-launch -y
catkin_make
roscd astra_camera && ./src/ros_astra_camera/scripts/create_udev_rules

echo 'Done.'
echo 'Ensure camera is correctly set by in `/src/usb_cam/launch/usb_cam-test.launch` line 3 `<param name="video_device" value="/dev/video0" />` (default)'
