#!/bin/bash

trap 'exit' ERR

# Install HandBrake
add-apt-repository -y ppa:stebbins/handbrake-releases
echo 'Running `apt-get update`...'
apt-get update -qq
echo 'Running `apt-get install handbrake-gtk handbrake-cli`...'
apt-get install -qq handbrake-gtk handbrake-cli
