#!/bin/bash

trap 'exit' ERR

# This file is an optional install: only use it if you wish to view the H.265 video output of the change detector

# Install support for H.265
add-apt-repository -y ppa:mc3man/trusty-media
apt-get update
apt-get dist-upgrade -y
apt-get install -y vlc vlc-plugin-libde265
