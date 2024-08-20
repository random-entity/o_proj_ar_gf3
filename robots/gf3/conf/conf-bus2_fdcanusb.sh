#!/bin/bash

# Do NOT use this often!

moteus_tool -t 7,8,9,10,11,12 --write-config common.cfg
moteus_tool -t 7,12 --write-config saj.cfg
moteus_tool -t 8,10 --write-config dj_l.cfg
moteus_tool -t 9,11 --write-config dj_r.cfg
moteus_tool -t 7,8,9,10,11,12 --write-config conf-write.cfg
