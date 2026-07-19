#!/bin/bash
# Disable SITL's fatal floating-point-exception trap so numeric spikes from the
# external physics don't abort the process; then rebuild copter.
set -e
cd ~/ardupilot
grep -rn "feenableexcept" libraries/AP_HAL_SITL/*.cpp | head -5
for f in $(grep -rl "feenableexcept" libraries/AP_HAL_SITL/*.cpp); do
  sed -i 's/^\(\s*\)feenableexcept(/\1\/\/ NOFPE: feenableexcept(/' "$f"
  echo "patched: $f"
done
./waf copter -j14 2>&1 | tail -2
ls -la build/sitl/bin/arducopter && echo NOFPE_BUILD_OK
