#!/bin/bash

trap 'exit' ERR

# Upgrade PIP package to support emailing
pip install requests requests_oauthlib --upgrade
