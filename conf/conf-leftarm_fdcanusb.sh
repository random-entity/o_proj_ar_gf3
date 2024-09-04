#!/bin/bash

# Do NOT use this often!

moteus_tool -t 1,2,3,4,5,6 --write-config common.cfg
moteus_tool -t 1,6 --write-config sgn_saj.cfg
moteus_tool -t 2,4 --write-config sgn_dj_l.cfg
moteus_tool -t 3,5 --write-config sgn_dj_r.cfg
moteus_tool -t 1 --write-config pid_shoulder_z.cfg
moteus_tool -t 2,3 --write-config pid_shoulder_xy.cfg
moteus_tool -t 4,5 --write-config pid_elbow.cfg
moteus_tool -t 6 --write-config pid_wrist.cfg
moteus_tool -t 1 --write-config slip_shoulder_z.cfg
moteus_tool -t 2,3 --write-config slip_shoulder_xy.cfg
moteus_tool -t 4,5 --write-config slip_elbow.cfg
moteus_tool -t 6 --write-config slip_wrist.cfg
moteus_tool -t 1,2,3,4,5,6 --write-config conf-write.cfg
