#!/bin/bash

if [ $# -eq 0 ]; then
  echo "No arguments provided, running for targets 1 to 14."
  targets=$(seq 1 14)
else
  targets="$@"
fi

for i in $targets
do
  echo "Running zero.exp with target $i..."
  expect ./zero.exp $i
  if [ $? -eq 0 ]; then
    echo "SUCCESS"
  else
    echo "FAIL"
  fi
done
