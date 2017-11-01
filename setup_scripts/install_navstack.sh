#!/bin/bash

trap 'exit' ERR

if [[ $EUID > 0 ]] ; then
  echo "Please run as root." >&2
  exit 1
fi

echo "Installing amcl ros package"
apt-get -y isntall ros-indigo-amcl

echo "Installing move_base ros package"
apt-get -y install ros-indigo-move-base
