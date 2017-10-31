#!/bin/bash

set -e

# x is empty.
x=''

if [[ $EUID > 0 ]] ; then
  echo "Please run as root." >&2
  exit 1
fi

#install openBLAS for better performance than default BLAS
echo "installing openBLAS for better performance."
apt-get install libopenblas-dev liblapack-dev

echo "Switching to ~/Documents to install there."
cd ~/Documents/

echo "Downloading driver zip."
wget https://github.com/davisking/dlib/archive/master.zip -O dlib.zip
if [ ! -z "${SUDO_USER+x}" ]; then
  chown $SUDO_USER:$SUDO_USER dlib.zip
fi

echo "Unzipping and changing directory."
unzip dlib.zip
if [ ! -z "${SUDO_USER+x}" ]; then
  chown -R $SUDO_USER:$SUDO_USER dlib-master
fi
cd dlib-master

echo "Changing permissions and executing setup.py with avx and cuda flags."
chmod +x setup.py
python setup.py install --yes USE_AVX_INSTRUCTIONS --yes DLIB_USE_CUDA

