#!/bin/bash
while true; do
  read -r line
  if [[ -n "$line" ]]; then
    echo "$line"
  fi
done
