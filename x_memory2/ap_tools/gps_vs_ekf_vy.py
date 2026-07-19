#!/usr/bin/env python3
"""Compare GPS-derived lateral (body-y) velocity vs EKF (XKF1) during a window.
If EKF vy moves but GPS vy doesn't -> phantom velocity (vibration corrupting EKF).
Usage: python gps_vs_ekf_vy.py <bin> <t0> <t1>
"""
import sys, math, bisect
from pymavlink import DFReader

path, T0, T1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
log = DFReader.DFReader_binary(path)

att, ekf, gps = [], [], []
while True:
    m = log.recv_msg()
    if m is None: break
    t = m.get_type()
    if t not in ("ATT", "XKF1", "GPS"): continue
    ts = m.TimeUS / 1e6
    if ts < T0 - 3 or ts > T1 + 3: continue
    if t == "ATT":
        att.append((ts, m.Yaw))
    elif t == "XKF1":
        if getattr(m, "C", 0) != 0: continue
        ekf.append((ts, m.VN, m.VE))
    elif t == "GPS":
        if getattr(m, "I", 0) != 0: continue
        # Spd = ground speed m/s, GCrs = course deg
        vn = m.Spd * math.cos(math.radians(m.GCrs))
        ve = m.Spd * math.sin(math.radians(m.GCrs))
        gps.append((ts, vn, ve, m.Status, m.NSats))

att_t = [a[0] for a in att]
def yaw_at(ts):
    i = max(0, min(len(att) - 1, bisect.bisect(att_t, ts)))
    return math.radians(att[i][1])

def body(vn, ve, y):
    return vn * math.cos(y) + ve * math.sin(y), -vn * math.sin(y) + ve * math.cos(y)

print("GPS fix during window: status/nsats %s" % str(set((g[3], g[4]) for g in gps)))
print("%8s | %6s %6s | %6s %6s | %s" % ("t", "ekf_vx", "ekf_vy", "gps_vx", "gps_vy", "dvy(ekf-gps)"))
gps_t = [g[0] for g in gps]
for ts, vn, ve in ekf:
    if ts < T0 or ts > T1: continue
    y = yaw_at(ts)
    evx, evy = body(vn, ve, y)
    i = max(0, min(len(gps) - 1, bisect.bisect(gps_t, ts)))
    g = gps[i]
    gvx, gvy = body(g[1], g[2], y)
    print("%8.2f | %6.2f %6.2f | %6.2f %6.2f | %6.2f" % (ts, evx, evy, gvx, gvy, evy - gvy))
