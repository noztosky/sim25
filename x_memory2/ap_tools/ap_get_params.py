#!/usr/bin/env python3
"""Read back params by name. Usage: python ap_get_params.py NAME [NAME...]"""
import sys, time
from pymavlink import mavutil

names = sys.argv[1:]
m = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
m.wait_heartbeat(timeout=40)
for name in names:
    m.param_fetch_one(name)
    got = None
    t0 = time.time()
    while time.time() - t0 < 4:
        a = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if a and a.param_id.strip("\x00") == name:
            got = a.param_value; break
    print("  %-18s = %s" % (name, got))
