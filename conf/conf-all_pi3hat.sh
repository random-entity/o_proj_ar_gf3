#!/bin/bash

# Do NOT use this often!

moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6' -t 1,2,3,4,5,6 --write-config common.cfg
moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6' -t 1,6 --write-config sgn_saj.cfg
moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6' -t 2,4 --write-config sgn_dj_l.cfg
moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6' -t 3,5 --write-config sgn_dj_r.cfg
moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6' -t 1 --write-config pid_shoulder_z.cfg
moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6' -t 2,3 --write-config pid_shoulder_xy.cfg
moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6' -t 4,5 --write-config pid_elbow.cfg
moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6' -t 6 --write-config pid_wrist.cfg
moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6' -t 1 --write-config slip_shoulder_z.cfg
moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6' -t 2,3 --write-config slip_shoulder_xy.cfg
moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6' -t 4,5 --write-config slip_elbow.cfg
moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6' -t 6 --write-config slip_wrist.cfg
moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6' -t 1,2,3,4,5,6 --write-config conf-write.cfg

moteus_tool --pi3hat-cfg '2=7,8,9,10,11,12' -t 7,8,9,10,11,12 --write-config common.cfg
moteus_tool --pi3hat-cfg '2=7,8,9,10,11,12' -t 7,12 --write-config sgn_saj.cfg
moteus_tool --pi3hat-cfg '2=7,8,9,10,11,12' -t 8,10 --write-config sgn_dj_l.cfg
moteus_tool --pi3hat-cfg '2=7,8,9,10,11,12' -t 9,11 --write-config sgn_dj_r.cfg
moteus_tool --pi3hat-cfg '2=7,8,9,10,11,12' -t 7 --write-config pid_shoulder_z.cfg
moteus_tool --pi3hat-cfg '2=7,8,9,10,11,12' -t 8,9 --write-config pid_shoulder_xy.cfg
moteus_tool --pi3hat-cfg '2=7,8,9,10,11,12' -t 10,11 --write-config pid_elbow.cfg
moteus_tool --pi3hat-cfg '2=7,8,9,10,11,12' -t 12 --write-config pid_wrist.cfg
moteus_tool --pi3hat-cfg '2=7,8,9,10,11,12' -t 7 --write-config slip_shoulder_z.cfg
moteus_tool --pi3hat-cfg '2=7,8,9,10,11,12' -t 8,9 --write-config slip_shoulder_xy.cfg
moteus_tool --pi3hat-cfg '2=7,8,9,10,11,12' -t 10,11 --write-config slip_elbow.cfg
moteus_tool --pi3hat-cfg '2=7,8,9,10,11,12' -t 12 --write-config slip_wrist.cfg
moteus_tool --pi3hat-cfg '2=7,8,9,10,11,12' -t 7,8,9,10,11,12 --write-config conf-write.cfg

moteus_tool --pi3hat-cfg '3=13,14' -t 13,14 --write-config conf-write.cfg
moteus_tool --pi3hat-cfg '3=13,14' -t 13 --write-config sgn_dj_l.cfg
moteus_tool --pi3hat-cfg '3=13,14' -t 14 --write-config sgn_dj_r.cfg
moteus_tool --pi3hat-cfg '3=13,14' -t 13,14 --write-config pid_neck.cfg
moteus_tool --pi3hat-cfg '3=13,14' -t 13,14 --write-config slip_neck.cfg
moteus_tool --pi3hat-cfg '3=13,14' -t 13,14 --write-config conf-write.cfg
