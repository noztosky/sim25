#!/usr/bin/env python3
import sys
from pymavlink import mavutil
port = sys.argv[1] if len(sys.argv) > 1 else "5762"
try:
    m = mavutil.mavlink_connection("tcp:127.0.0.1:%s" % port)
    hb = m.wait_heartbeat(timeout=8)
    print("HEARTBEAT OK on %s (type=%s autopilot=%s)" % (port, hb.type, hb.autopilot))
except Exception as e:
    print("NO HEARTBEAT on %s: %s" % (port, e))
