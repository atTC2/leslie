#!/bin/bash

# This file is an optional install: only use it if you wish to view the H.265 video output of the change detector

# Install support for H.265
sudo add-apt-repository ppa:mc3man/trusty-media
sudo apt-get update
sudo apt-get dist-upgrade
sudo apt-get install vlc vlc-plugin-libde265
