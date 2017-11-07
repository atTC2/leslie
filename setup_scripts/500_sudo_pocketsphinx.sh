#!/bin/bash

trap 'exit' ERR

echo -n 'Running `apt-get install gstreamer0.10-gconf gstreamer0.10-pocketsphinx ros-indigo-pocketsphinx ros-indigo-audio-common libasound2`... '
apt-get install -qq gstreamer0.10-gconf gstreamer0.10-pocketsphinx ros-indigo-pocketsphinx ros-indigo-audio-common libasound2
echo 'done'
