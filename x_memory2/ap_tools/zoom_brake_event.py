#!/usr/bin/env python3
"""Zoom into a time window of the real .bin: dump ATT/RATE/XKF1/RCOU/VIBE
to CSV and print a per-0.2s digest. Usage:
  python zoom_brake_event.py <bin> <t0> <t1> [out.csv]
"""
import sys, math, csv, bisect
from pymavlink import DFReader

path, T0, T1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
out = sys.argv[4] if len(sys.argv) > 4 else "logs/silapp/real_zoom_%d_%d.csv" % (T0, T1)

log = DFReader.DFReader_binary(path)
att, rate, vel, rcou, vibe = [], [], [], [], []
while True:
    m = log.recv_msg()
    if m is None: break
    t = m.get_type()
    if t not in ("ATT", "RATE", "XKF1", "RCOU", "VIBE"): continue
    ts = m.TimeUS / 1e6
    if ts < T0 - 5 or ts > T1 + 5: continue
    if t == "ATT":
        att.append((ts, m.Roll, m.DesRoll, m.Pitch, m.DesPitch, m.Yaw))
    elif t == "RATE":
        rate.append((ts, m.R, m.P))  # deg/s in dataflash
    elif t == "XKF1":
        if getattr(m, "C", 0) != 0: continue
        vel.append((ts, m.VN, m.VE))
    elif t == "RCOU":
        rcou.append((ts, m.C1, m.C2, m.C3, m.C4))
    elif t == "VIBE":
        vibe.append((ts, m.VibeX, m.VibeY, m.VibeZ))

att_t = [a[0] for a in att]
def near(series, ts, idx_t=0):
    if not series: return None
    ts_list = [s[idx_t] for s in series]
    i = max(0, min(len(series) - 1, bisect.bisect(ts_list, ts)))
    return series[i]

rows = []
for a in att:
    ts, roll, desroll, pitch, despitch, yaw = a
    if ts < T0 or ts > T1: continue
    v = near(vel, ts); r = near(rate, ts); o = near(rcou, ts); vb = near(vibe, ts)
    y = math.radians(yaw)
    vxb = v[1] * math.cos(y) + v[2] * math.sin(y) if v else 0
    vyb = -v[1] * math.sin(y) + v[2] * math.cos(y) if v else 0
    rows.append([round(ts, 2), round(roll, 2), round(desroll, 2), round(pitch, 2),
                 round(despitch, 2), round(vxb, 2), round(vyb, 2),
                 round(r[1], 1) if r else 0, round(r[2], 1) if r else 0,
                 o[1] if o else 0, o[2] if o else 0, o[3] if o else 0, o[4] if o else 0,
                 round(vb[2], 1) if vb else 0])

with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "roll", "desroll", "pitch", "despitch", "vx_body", "vy_body",
                "rollrate", "pitchrate", "c1", "c2", "c3", "c4", "vibey"])
    w.writerows(rows)
print("wrote %d rows -> %s" % (len(rows), out))

# digest every 0.2s
step = T0
for row in rows:
    if row[0] >= step:
        print("t=%7.2f roll=%6.2f des=%6.2f pitch=%6.2f vx=%5.2f vy=%5.2f "
              "rr=%6.1f | M1-4 %4d %4d %4d %4d | vibY=%4.1f" %
              (row[0], row[1], row[2], row[3], row[5], row[6], row[7],
               row[9], row[10], row[11], row[12], row[13]))
        step = row[0] + 0.2
