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

wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1404/x86_64/cuda-repo-ubuntu1404_7.5-18_amd64.deb
dpkg -i cuda-repo-ubuntu1404_7.5-18_amd64.deb
rm cuda-repo-ubuntu1404_7.5-18_amd64.deb
echo 'export CUDA_HOME=/usr/local/cuda
export CUDA_ROOT=/usr/local/cuda
export PATH=$PATH:$CUDA_ROOT/bin:$HOME/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CUDA_ROOT/lib64
' >> ~/.bashrc

# Let terminal know of the changes to the .bashrc file
source .bashrc

apt-get update 

# y flag just says yes to all prompts
apt-get install -y cuda

# Check if installation is successful by running the next line
# nvcc -V

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

