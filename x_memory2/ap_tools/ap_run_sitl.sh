#!/bin/bash
# Run ArduPilot SITL against AirSim (mirrored networking -> localhost both ways).
# Logs to ~/sitl_run.log ; serial0 available for MAVProxy on tcp:127.0.0.1:5760
cd ~/ardupilot
pkill -f 'bin/arducopter' 2>/dev/null
sleep 1
rm -f ~/sitl_run.log
nohup build/sitl/bin/arducopter \
    --model airsim \
    --speedup 1 \
    --defaults Tools/autotest/default_params/copter.parm,Tools/autotest/default_params/airsim-quadX.parm \
    -I0 > ~/sitl_run.log 2>&1 &
echo "SITL started pid $!"
sleep 4
tail -12 ~/sitl_run.log
