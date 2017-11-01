#!/bin/bash

trap 'exit' ERR

curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
# `apt-get update` is done in the script.
sudo apt-get install -y git-lfs
git lfs install
