#!/bin/bash

trap 'exit' ERR

# This file is an optional install: only use it if you wish to view the H.265 video output of the change detector

# Install support for H.265
add-apt-repository -y ppa:mc3man/trusty-media
echo 'Running `apt-get update`...'
apt-get update -qq
echo 'Running `apt-get dist-upgrade`...'
apt-get dist-upgrade -qq
echo 'Running `apt-get install vlc vlc-plugin-libde265`...'
apt-get install -qq vlc vlc-plugin-libde265
