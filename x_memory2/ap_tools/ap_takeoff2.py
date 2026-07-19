#!/usr/bin/env python3
"""Takeoff attempt with EOF-safe monitoring (stops on link loss)."""
import time
from pymavlink import mavutil

m = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
m.wait_heartbeat(timeout=40)
m.mav.request_data_stream_send(m.target_system, m.target_component,
                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 6, 1)
# wait EKF position
t0 = time.time()
while time.time() - t0 < 90:
    msg = m.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=5)
    if msg and msg.flags & 16:
        print("EKF pos OK (%.0fs)" % (time.time() - t0)); break
m.set_mode_apm("GUIDED"); time.sleep(1)
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
m.recv_match(type="COMMAND_ACK", blocking=True, timeout=4)
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
print("takeoff sent")
t0 = time.time(); mx = 0; seen = set(); eof = 0; last = 0
while time.time() - t0 < 40:
    try:
        msg = m.recv_match(type=["GLOBAL_POSITION_INT", "STATUSTEXT", "ATTITUDE"],
                           blocking=True, timeout=3)
    except Exception as e:
        print("LINK LOST:", e); break
    if msg is None:
        eof += 1
        if eof >= 3:
            print("LINK DEAD (no data 9s) at t=%.1f — SITL crashed?" % (time.time() - t0)); break
        continue
    eof = 0
    t = msg.get_type()
    if t == "GLOBAL_POSITION_INT":
        a = msg.relative_alt / 1000.0; mx = max(mx, a)
        if time.time() - t0 - last >= 2:
            last = time.time() - t0
            print("  t=%4.1f alt=%.2f" % (last, a))
    elif t == "STATUSTEXT" and msg.text not in seen:
        seen.add(msg.text); print("  STATUS:", msg.text)
print("MAX ALT = %.2f" % mx)
