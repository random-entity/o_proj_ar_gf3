#!/bin/bash

for i in {1..14}
do
  echo "Running zero.exp with target $i..."
  expect ./zero.exp $i
  if [ $? -eq 0 ]; then
    echo "SUCCESS"
  else
    echo "FAIL"
  fi
done
