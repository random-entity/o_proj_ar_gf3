#!/bin/bash

cd /home/ubuntu/tek/repos/o_proj_ar_gf3/bin
sudo ./gf3

# Don't run this on host(nuc), but run `./gf3_launch.sh`
# within the /home/ubuntu/ directory after accessing ubuntu@som-rpi4-2 via ssh.
# This file is linked against to ubuntu@som-rpi4-2's `~/gf3_launch.sh`.
