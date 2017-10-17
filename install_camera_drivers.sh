#!/bin/bash

if [[ $EUID > 0 ]]
  then echo "Please run as root"
  exit
fi
echo "Downloading driver zip to current directory."
wget http://www.orbbec3d.net/Tools_SDK_OpenNI/2-Linux.zip

echo "Unzipping and changing directory."
unzip 2-Linux.zip
cd 2-Linux
unzip OpenNI-Linux-x64-2.3.zip
cd OpenNI-Linux-x64-2.3

echo "Changing permissions and executing install script."
chmod +x install.sh
./install.sh

echo "You can check whether this has installed correctly by connecting the camera and running SampleView or SampleRead in 2-Linux/OpenNI-Linux-x64-2.3/Samples/Bin/"
