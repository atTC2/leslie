#!/bin/bash

trap 'exit' ERR

if [[ $EUID > 0 ]] ; then
  echo 'Please run as root.' >&2
  exit 1
fi

echo -n 'Installing amcl ros package and move_base ros package...'
apt-get install -qq ros-indigo-amcl ros-indigo-move-base
echo 'done'
