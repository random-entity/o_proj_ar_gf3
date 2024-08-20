#!/bin/bash

# Do NOT use this often!

moteus_tool -t 1,2,3,4,5,6 --write-config common.cfg
moteus_tool -t 1,6 --write-config saj.cfg
moteus_tool -t 2,4 --write-config dj_l.cfg
moteus_tool -t 3,5 --write-config dj_r.cfg
moteus_tool -t 1,2,3,4,5,6 --write-config conf-write.cfg
