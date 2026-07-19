#!/usr/bin/env python3
"""Set SITL vibration-injection + harmonic-notch params (persist in eeprom).

Usage: python ap_set_vibe.py [acc_rnd] [gyr_rnd] [notch_on]
Defaults: acc_rnd=10 gyr_rnd=40 notch_on=1
Real K20 evidence: VibeY mean 7.2 m/s2 peak 22.3, notch ESC-mode 28 Hz.
SITL: sine amplitude = SIM_ACC_RND (m/s2) / SIM_GYR_RND*throttle (deg/s),
gated by motors_on. MODE 3(ESC) has no telemetry in SITL -> use MODE 1
(throttle) with REF = real MOT_THST_HOVER 0.165.
"""
import sys, time
from pymavlink import mavutil

ACC = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
GYR = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0
NOTCH = int(sys.argv[3]) if len(sys.argv) > 3 else 1

PARAMS = {
    "SIM_VIB_FREQ_X": 28.0,
    "SIM_VIB_FREQ_Y": 28.0,
    "SIM_VIB_FREQ_Z": 28.0,
    "SIM_ACC_RND": ACC,
    "SIM_GYR_RND": GYR,
    "INS_HNTCH_ENABLE": float(NOTCH),
    "INS_HNTCH_MODE": 1.0,     # throttle-based tracking (ESC telem absent in SITL)
    "INS_HNTCH_REF": 0.165,    # real MOT_THST_HOVER
    "INS_HNTCH_FREQ": 28.0,
    "INS_HNTCH_BW": 12.0,
    "INS_HNTCH_ATT": 40.0,
    "INS_HNTCH_HMNCS": 1.0,
}

m = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
m.wait_heartbeat(timeout=40)
print("connected")
ok = 0
for name, val in PARAMS.items():
    m.param_set_send(name, val)
    a = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
    got = None
    t0 = time.time()
    while time.time() - t0 < 3:
        if a and a.param_id.strip("\x00") == name:
            got = a.param_value; break
        a = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
    status = "OK" if (got is not None and abs(got - val) < 1e-3) else "FAIL(got %s)" % got
    if status == "OK": ok += 1
    print("  %-18s = %-8g %s" % (name, val, status))
print("%d/%d set" % (ok, len(PARAMS)))
