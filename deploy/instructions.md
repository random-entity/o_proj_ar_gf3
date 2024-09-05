1. Check nuc and RPi's router connection via LAN cable.
2. Power on motors.
3. Power on RPi.
4. Power on nuc.
5. On nuc, access RPi using `ssh-access-rpi.sh`.
6. On RPi's `/home/ubuntu` accessed by nuc, launch Parasite using `./gf3_launch.sh` if not auto-launched at RPi startup.
7. To monitor pose file save, launch `deploy/monitor-pose-save` on RPi from another ssh session from nuc. Pose file load log will print at console running Parasite.
8. On nuc, open main host Pd patch `src/host/pd/main.pd`.
9. Check default max trq/vel/acc loaded as `16`, command output velocities as `0.1`.
10. Check zero position on `OutVel` Mode.
11. Start timed sequence of poses and slide cues.