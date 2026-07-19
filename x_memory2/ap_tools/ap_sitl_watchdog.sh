#!/bin/bash
# SITL watchdog: keep arducopter running; auto-restart if it exits for any reason.
# Run this in a PERSISTENT Windows background task (wsl -e bash <this>).
cd ~/ardupilot
pkill -f 'bin/arducopter' 2>/dev/null
sleep 1
n=0
while true; do
  n=$((n+1))
  echo "=== SITL start #$n $(date +%H:%M:%S) ==="
  build/sitl/bin/arducopter \
      --model airsim \
      --speedup 1 \
      --defaults Tools/autotest/default_params/copter.parm,Tools/autotest/default_params/airsim-quadX.parm,/mnt/d/xlab/sim25/x_memory2/build/k20_real_sitl.parm \
      --serial0 tcp:0:nowait \
      --serial4 udpclient:127.0.0.1:14550 \
      --serial5 tcp:5:nowait \
      -I0 2>&1 | grep --line-buffered -vE '^Loaded defaults|^FPS' | tee -a ~/sitl_watchdog.log &
  AP_PID=$!
  wait $AP_PID
  echo "=== SITL exited (run #$n), restarting in 3s ==="
  sleep 3
done
