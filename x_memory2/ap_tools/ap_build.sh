#!/bin/bash
# ArduPilot SITL build for WSL (called from Windows via wsl.exe)
set -e
cd ~
echo "[1/4] pip deps (no sudo)"
pip3 install --user -q "empy==3.3.4" pexpect future intelhex 2>&1 | tail -1 || true
echo "[2/4] clone ArduPilot Copter-4.5.7 (shallow)"
if [ ! -d ardupilot ]; then
  git clone -q --depth 1 --branch Copter-4.5.7 https://github.com/ArduPilot/ardupilot.git
fi
cd ardupilot
echo "[3/4] submodules"
git submodule update --init --recursive 2>&1 | tail -2
echo "[4/4] waf configure + build"
./waf configure --board sitl 2>&1 | tail -3
./waf copter -j14 2>&1 | tail -4
ls -la build/sitl/bin/arducopter && echo BUILD_OK
