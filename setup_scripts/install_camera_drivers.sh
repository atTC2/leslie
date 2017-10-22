#!/bin/bash

if [[ $EUID > 0 ]] ; then
  echo "Please run as root." >&2
  exit
fi

echo "Switching to ~/Documents to install there."
cd ~/Documents/

echo "Downloading driver zip."
wget http://www.orbbec3d.net/Tools_SDK_OpenNI/2-Linux.zip
chown $SUDO_USER:$SUDO_USER 2-Linux.zip 

echo "Unzipping and changing directory."
unzip 2-Linux.zip
cd 2-Linux
unzip OpenNI-Linux-x64-2.3.zip
chown -R $SUDO_USER:$SUDO_USER ../2-Linux 
cd OpenNI-Linux-x64-2.3

echo "Changing permissions and executing install script."
chmod +x install.sh
./install.sh
chmod +x Samples/Bin/SimpleRead
chmod +x Samples/Bin/SimpleViewer

echo "You can check whether this has installed correctly by connecting the camera and running SimpleView or SimpleRead in 2-Linux/OpenNI-Linux-x64-2.3/Samples/Bin/"
