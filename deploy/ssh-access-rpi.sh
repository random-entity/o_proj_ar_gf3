#!/bin/bash

while true; do
  echo "Try connecting to RPi..."
  ssh ubuntu@192.168.0.103
  if [ $? -ne 0 ]; then
    echo "Failed. retrying in 5 seconds..."
    sleep 5
  else
    break
  fi
done

# Access som-rpi4-2 with this script, then run
# `./gf3_launch.sh` within the /home/ubuntu/ directory.
