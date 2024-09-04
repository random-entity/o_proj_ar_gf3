#!/bin/bash

if [ $# -eq 0 ]; then
  echo "No arguments provided, running for targets 1 to 14."
  targets=$(seq 1 14)
else
  targets="$@"
fi

for i in $targets
do
  echo "Dumping conf for target $i..."
  moteus_tool -t $i --dump-config > $i.cfg
  cat $i.cfg | perl -n -e 'print "conf set $_" if /\S/' > $i-write.cfg
  echo "conf write" >> $i-write.cfg
  if [ $? -eq 0 ]; then
    echo "SUCCESS"
  else
    echo "FAIL"
  fi
done
