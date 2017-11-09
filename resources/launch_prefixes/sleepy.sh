#!/bin/bash

trap 'exit' ERR

time=$(bc -l <<< "$1*$2")

sleep "${time}s"

shift 2

exec "$@"
