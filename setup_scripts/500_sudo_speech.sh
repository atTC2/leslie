#!/bin/bash

trap 'exit' ERR

echo -n 'Running `apt-get install libttspico-utils sox`... '
apt-get install -qq libttspico-utils sox
echo 'done'
