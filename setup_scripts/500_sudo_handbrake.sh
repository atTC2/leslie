#!/bin/bash

trap 'exit' ERR

# Install HandBrake
add-apt-repository -y ppa:stebbins/handbrake-releases
apt-get update
apt-get install -y handbrake-gtk
apt-get install -y handbrake-cli
