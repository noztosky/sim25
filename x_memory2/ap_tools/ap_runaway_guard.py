#!/usr/bin/env python3
"""Runaway guard: watches the SITL vehicle and FORCE-DISARMS the instant it goes crazy.

Triggers (any):
  |roll| or |pitch| > 70 deg
  |gyro| any axis   > 8 rad/s
  |climb rate|      > 8 m/s
  relative alt      > 60 m  (or < -5 m)

Runs forever on SERIAL2 (tcp:5763) so Mission Planner (5762) and test scripts (5760)
are unaffected. Reconnects if SITL restarts.
"""
import time
from pymavlink import mavutil

PORT = "tcp:127.0.0.1:5763"
LIM_ATT = 70.0       # deg
LIM_GYRO = 8.0       # rad/s
LIM_CLIMB = 8.0      # m/s
LIM_ALT = 60.0       # m
COOLDOWN = 6.0       # s between kills

def log(s):
    print(time.strftime("[%H:%M:%S]"), s, flush=True)

def guard_once():
    m = mavutil.mavlink_connection(PORT)
    m.wait_heartbeat(timeout=60)
    log("guard connected on 5763")
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    last_kill = 0.0
    while True:
        msg = m.recv_match(type=["ATTITUDE", "GLOBAL_POSITION_INT", "VFR_HUD"],
                           blocking=True, timeout=5)
        if msg is None:
            # link silent — SITL likely restarting
            raise ConnectionError("link silent")
        t = msg.get_type()
        reason = None
        if t == "ATTITUDE":
            import math
            r = abs(math.degrees(msg.roll)); p = abs(math.degrees(msg.pitch))
            g = max(abs(msg.rollspeed), abs(msg.pitchspeed), abs(msg.yawspeed))
            if r > LIM_ATT or p > LIM_ATT:
                reason = "attitude r=%.0f p=%.0f deg" % (r, p)
            elif g > LIM_GYRO:
                reason = "gyro %.1f rad/s" % g
        elif t == "GLOBAL_POSITION_INT":
            alt = msg.relative_alt / 1000.0
            if alt > LIM_ALT or alt < -5.0:
                reason = "alt %.1f m" % alt
        elif t == "VFR_HUD":
            if abs(msg.climb) > LIM_CLIMB:
                reason = "climb %.1f m/s" % msg.climb
        if reason and time.time() - last_kill > COOLDOWN:
            last_kill = time.time()
            log("!! RUNAWAY: %s -> FORCE DISARM" % reason)
            for _ in range(3):
                m.mav.command_long_send(m.target_system, m.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                        0, 0, 21196, 0, 0, 0, 0, 0)
                time.sleep(0.2)

while True:
    try:
        guard_once()
    except Exception as e:
        log("guard reconnecting (%s)" % e)
        time.sleep(3)
