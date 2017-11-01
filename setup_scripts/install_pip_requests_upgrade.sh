#!/bin/bash

trap 'exit' ERR

# Upgrade PIP package to support emailing
sudo pip install requests --upgrade
