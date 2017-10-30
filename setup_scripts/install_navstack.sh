#!/bin/bash

if [[ $EUID > 0 ]] ; then
  echo "Please run as root." >&2
  exit
fi

printf "Installing move_base ros package and dependencies"
apt-get -y install ros-indigo-move-base
