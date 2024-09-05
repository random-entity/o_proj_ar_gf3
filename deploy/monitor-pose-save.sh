#!/bin/bash

# Use this in RPi to monitor pose file loads

echo "Monitoring file changes in /home/ubuntu/tek/repos/o_proj_ar_gf3/poses"

MONITOR_DIR=/home/ubuntu/tek/repos/o_proj_ar_gf3/poses

# Check if the directory exists
if [ ! -d "$MONITOR_DIR" ]; then
  echo "Error: Directory $MONITOR_DIR does not exist."
  exit 1
fi

# Check if inotifywait command is installed
if ! command -v inotifywait &> /dev/null; then
  echo "Error: inotifywait is not installed. Install it with 'sudo apt install inotify-tools'."
  exit 1
fi

# Monitor the directory for creation and modification events
inotifywait -m -e create -e modify --format '%e %f' "$MONITOR_DIR" | while read event file; do
  if [[ "$event" == *"CREATE"* ]]; then
    echo "new file: $file"
  elif [[ "$event" == *"MODIFY"* ]]; then
    echo "modified file: $file"
  fi
done
