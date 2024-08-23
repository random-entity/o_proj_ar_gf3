#!/bin/bash

# Do NOT use this often!

moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6;2=7,8,9,10,11,12;3=13,14' -t 1,2,3,4,5,6,7,8,9,10,11,12,13,14 --write-config common.cfg
moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6;2=7,8,9,10,11,12;3=13,14' -t 1,6,7,12 --write-config saj.cfg
moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6;2=7,8,9,10,11,12;3=13,14' -t 2,4,8,10,13 --write-config dj_l.cfg
moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6;2=7,8,9,10,11,12;3=13,14' -t 3,5,9,11,14 --write-config dj_r.cfg
moteus_tool --pi3hat-cfg '1=1,2,3,4,5,6;2=7,8,9,10,11,12;3=13,14' -t 1,2,3,4,5,6,7,8,9,10,11,12,13,14 --write-config conf-write.cfg
