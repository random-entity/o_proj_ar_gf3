#!/bin/bash

# Do NOT use this often!

moteus_tool -t 13,14 --write-config common.cfg
moteus_tool -t 13 --write-config sgn_dj_l.cfg
moteus_tool -t 14 --write-config sgn_dj_r.cfg
moteus_tool -t 13,14 --write-config pid_neck.cfg
moteus_tool -t 13,14 --write-config conf-write.cfg
