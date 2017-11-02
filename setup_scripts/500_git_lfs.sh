#!/bin/bash

trap 'exit' ERR

curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
# `apt-get update` is done in the script.
echo -n 'Running `apt-get install git-lfs`... '
sudo apt-get install -qq git-lfs
echo 'done'
git lfs install
