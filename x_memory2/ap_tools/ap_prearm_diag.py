#!/usr/bin/env python3
"""Diagnose 'Need Position Estimate': read EKF flags + the params that usually break
SITL after loading real-vehicle (RTK) parameters."""
import sys, time
from pymavlink import mavutil

port = sys.argv[1] if len(sys.argv) > 1 else "5760"
m = mavutil.mavlink_connection("tcp:127.0.0.1:%s" % port)
m.wait_heartbeat(timeout=15)
print("connected on", port)
m.mav.request_data_stream_send(m.target_system, m.target_component,
                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

# fetch the usual suspects
SUSPECTS = ["EK3_SRC1_POSXY", "EK3_SRC1_VELXY", "EK3_SRC1_POSZ", "EK3_SRC1_YAW",
            "GPS_TYPE", "GPS_TYPE2", "GPS_AUTO_CONFIG", "GPS_RATE_MS",
            "COMPASS_USE", "COMPASS_USE2", "COMPASS_USE3", "COMPASS_ENABLE",
            "AHRS_EKF_TYPE", "EK3_ENABLE", "EK2_ENABLE", "AHRS_GPS_USE",
            "EK3_GPS_CHECK", "FS_EKF_THRESH", "SIM_GPS_DISABLE", "SIM_GPS_TYPE"]
vals = {}
for name in SUSPECTS:
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
    if msg and msg.param_id == name:
        vals[name] = msg.param_value
print("\n=== SUSPECT PARAMS ===")
for k in SUSPECTS:
    print("  %-16s = %s" % (k, vals.get(k, "(?)")))

# EKF flags + GPS + statustext for 12 s
print("\n=== LIVE (12 s) ===")
t0 = time.time()
seen = set()
ekf_flags = None
gps = None
while time.time() - t0 < 12:
    msg = m.recv_match(type=["STATUSTEXT", "EKF_STATUS_REPORT", "GPS_RAW_INT"],
                       blocking=True, timeout=3)
    if msg is None:
        continue
    t = msg.get_type()
    if t == "STATUSTEXT" and msg.text not in seen:
        seen.add(msg.text)
        print("  STATUS:", msg.text)
    elif t == "EKF_STATUS_REPORT":
        ekf_flags = msg.flags
    elif t == "GPS_RAW_INT":
        gps = msg

if gps:
    print("\nGPS: fix=%d sats=%d hdop=%.2f" % (gps.fix_type, gps.satellites_visible, gps.eph / 100.0))
if ekf_flags is not None:
    names = [(1, "attitude"), (2, "velocity_horiz"), (4, "velocity_vert"),
             (8, "pos_horiz_rel"), (16, "pos_horiz_abs"), (32, "pos_vert_abs"),
             (64, "pos_vert_agl"), (128, "const_pos_mode"), (256, "pred_pos_horiz_rel"),
             (512, "pred_pos_horiz_abs")]
    print("EKF flags = 0x%x" % ekf_flags)
    for bit, nm in names:
        print("   %-18s %s" % (nm, "OK" if ekf_flags & bit else "--"))
