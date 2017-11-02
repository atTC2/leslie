#!/bin/bash

trap 'exit' ERR

# SciPy is needed for face_recognition. The rest are needed for SciPy.
echo 'Running `apt-get install gfortran libopenblas-dev liblapack-dev python-dev python-scipy`...'
apt-get install -qq gfortran libopenblas-dev liblapack-dev python-dev python-scipy

pip install face_recognition
