#!/bin/bash

# Do NOT use this often!

moteus_tool -t 7,8,9,10,11,12 --write-config common.cfg
moteus_tool -t 7,12 --write-config saj.cfg
moteus_tool -t 8,10 --write-config dj_l.cfg
moteus_tool -t 9,11 --write-config dj_r.cfg
moteus_tool -t 7 --write-config pid_shoulder_z.cfg
moteus_tool -t 8,9 --write-config pid_shoulder_xy.cfg
moteus_tool -t 10,11 --write-config pid_elbow.cfg
moteus_tool -t 12 --write-config pid_wrist.cfg
moteus_tool -t 7,8,9,10,11,12 --write-config conf-write.cfg
