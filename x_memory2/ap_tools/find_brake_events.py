#!/usr/bin/env python3
"""Scan a real .bin dataflash log for backward-flight -> brake events and
report roll behaviour in each brake window (hunting the 'tips sideways' symptom).

Body-frame velocity from XKF1 (VN,VE) rotated by ATT.Yaw.
Backward = body-x velocity < -SPEED_MIN sustained; brake = return to |v|<0.7.
Also reports forward->stop events for contrast.

Usage: python find_brake_events.py [path.bin]
"""
import sys, math
from pymavlink import DFReader

path = sys.argv[1] if len(sys.argv) > 1 else "x_memory2/build/real_autotune.bin"
SPEED_MIN = 2.0

log = DFReader.DFReader_binary(path)

att = []   # (t, roll, pitch, yaw, desroll)
vel = []   # (t, vn, ve)
rate = []  # (t, rollrate_dps)
modes = [] # (t, modename)
n = 0
while True:
    m = log.recv_msg()
    if m is None:
        break
    n += 1
    t = m.get_type()
    if t == "ATT":
        ts = m.TimeUS / 1e6
        att.append((ts, m.Roll, m.Pitch, m.Yaw, m.DesRoll))
    elif t == "XKF1":
        if getattr(m, "C", 0) != 0:
            continue
        vel.append((m.TimeUS / 1e6, m.VN, m.VE))
    elif t == "RATE":
        rate.append((m.TimeUS / 1e6, m.R))  # dataflash RATE.R is deg/s already
    elif t == "MODE":
        modes.append((m.TimeUS / 1e6, m.asText if hasattr(m, "asText") else str(m.Mode)))
print("parsed %d msgs: ATT %d, XKF1 %d, RATE %d" % (n, len(att), len(vel), len(rate)))

# body-x velocity series (nearest yaw)
import bisect
att_t = [a[0] for a in att]
def yaw_at(ts):
    i = bisect.bisect(att_t, ts)
    i = max(0, min(len(att) - 1, i))
    return att[i][3]

vx = []  # (t, vx_body, vy_body)
for ts, vn, ve in vel:
    y = math.radians(yaw_at(ts))
    vx.append((ts, vn * math.cos(y) + ve * math.sin(y), -vn * math.sin(y) + ve * math.cos(y)))

def mode_at(ts):
    cur = "?"
    for tm, name in modes:
        if tm > ts: break
        cur = name
    return cur

def window_stats(t0, t1):
    ro = [(a[1], a[1] - a[4]) for a in att if t0 <= a[0] <= t1]
    rr = [r[1] for r in rate if t0 <= r[0] <= t1]
    vy_w = [v[2] for v in vx if t0 <= v[0] <= t1]
    if not ro: return None
    return (max(abs(r[0]) for r in ro), max(abs(r[1]) for r in ro),
            max(abs(r) for r in rr) if rr else 0.0,
            max(abs(v) for v in vy_w) if vy_w else 0.0)

# find sustained motion segments then brake
def find_events(sign):
    events = []
    in_seg = False; seg_start = 0; peak = 0
    for ts, vxb, _ in vx:
        v = vxb * sign
        if v > SPEED_MIN:
            if not in_seg:
                in_seg = True; seg_start = ts; peak = v
            peak = max(peak, v)
        elif in_seg and v < 0.7:
            if ts - seg_start > 0.8:
                events.append((seg_start, ts, peak))
            in_seg = False; peak = 0
    return events

for label, sign in [("BACKWARD", -1), ("FORWARD", 1)]:
    evs = find_events(sign)
    print("\n%s->stop events: %d" % (label, len(evs)))
    for t0, t1, pk in evs:
        st = window_stats(t1 - 0.5, t1 + 3.0)
        if st is None: continue
        roll_pk, rollerr_pk, rr_pk, vy_pk = st
        print("  t=%7.1f-%7.1f  v_pk=%.1f m/s  mode=%-8s | brake: |roll|=%5.1f "
              "rollerr=%5.1f rate=%6.1f dps  |vy|=%.1f" %
              (t0, t1, pk, mode_at(t1), roll_pk, rollerr_pk, rr_pk, vy_pk))
