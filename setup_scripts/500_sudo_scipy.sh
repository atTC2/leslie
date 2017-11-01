#!/bin/bash

trap 'exit' ERR

apt install gfortran libopenblas-dev liblapack-dev python-dev
pip install scipy
