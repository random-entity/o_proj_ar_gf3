1. Power on motors.
2. Power on RPi.
3. Power on nuc.
4. On nuc, access RPi using `ssh-access-rpi.sh`.
5. On RPi's `/home/ubuntu` accessed by nuc, launch Parasite using `./gf3_launch.sh`.
6. On nuc, open main host Pd patch `src/host/pd/main.pd`.
7. Check default max trq/vel/acc loaded as `16`, command output velocities as `0.1`.
8. Check zero position on `OutVel` Mode.
9. Start timed sequence of poses and slide cues.