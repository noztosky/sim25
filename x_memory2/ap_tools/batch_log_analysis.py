#!/usr/bin/env python3
"""Batch precision analysis of a parameter-sweep test session (.bin logs).

Per log: params-of-interest, param DIFF vs previous log (reveals the test
sequence), hover motor split + attitude (CG / mount-offset check), backward &
forward brake events with roll behaviour, vibe levels.
DJI-X (FRAME_TYPE 13): C1=FR C2=RR C3=RL C4=FL; right=C1+C2, rear=C2+C3.
Usage: python batch_log_analysis.py <file1.bin> [file2.bin ...]
"""
import sys, math, bisect
from collections import defaultdict
from pymavlink import DFReader

MOT_PWM_MIN, MOT_PWM_MAX, EXPO = 1050.0, 1950.0, 0.75
MASS_G = 18.0 * 9.81

def thrust_norm(pwm):
    a = max(0.0, min(1.0, (pwm - MOT_PWM_MIN) / (MOT_PWM_MAX - MOT_PWM_MIN)))
    return (1 - EXPO) * a + EXPO * a * a

def analyze(path):
    log = DFReader.DFReader_binary(path)
    params = {}
    att, vel, rate = [], [], []
    hover = defaultdict(list)
    vibe_y = []
    modes = []
    last_v = (9.0, 9.0); last_att = (0.0, 0.0, 0.0)
    t_first = t_last = None
    while True:
        m = log.recv_msg()
        if m is None: break
        t = m.get_type()
        if t == "PARM":
            params[m.Name] = m.Value; continue
        if not hasattr(m, "TimeUS"): continue
        ts = m.TimeUS / 1e6
        if t_first is None: t_first = ts
        t_last = ts
        if t == "ATT":
            last_att = (m.Roll, m.Pitch, m.Yaw)
            att.append((ts, m.Roll, m.Pitch, m.Yaw, m.DesRoll))
        elif t == "XKF1":
            if getattr(m, "C", 0) != 0: continue
            last_v = (m.VN, m.VE)
            vel.append((ts, m.VN, m.VE))
        elif t == "RATE":
            rate.append((ts, m.R))
        elif t == "VIBE":
            vibe_y.append(m.VibeY)
        elif t == "MODE":
            modes.append(getattr(m, "asText", None) or str(m.Mode))
        elif t == "RCOU":
            vn, ve = last_v
            if abs(vn) < 0.3 and abs(ve) < 0.3 and m.C1 > 1300:
                hover["C1"].append(m.C1); hover["C2"].append(m.C2)
                hover["C3"].append(m.C3); hover["C4"].append(m.C4)
                hover["roll"].append(last_att[0]); hover["pitch"].append(last_att[1])

    out = {"params": params, "dur": (t_last - t_first) if t_first else 0,
           "modes": sorted(set(modes))}
    # hover stats
    if hover["C1"] and len(hover["C1"]) > 50:
        mean = {k: sum(hover[k]) / len(hover[k]) for k in ("C1", "C2", "C3", "C4")}
        tn = {k: thrust_norm(mean[k]) for k in mean}
        tot = sum(tn.values())
        thr = {k: MASS_G * tn[k] / tot for k in tn}
        out["hover"] = {
            "n": len(hover["C1"]), "pwm": mean,
            "right_left_N": (thr["C1"] + thr["C2"]) - (thr["C3"] + thr["C4"]),
            "rear_front_N": (thr["C2"] + thr["C3"]) - (thr["C1"] + thr["C4"]),
            "roll": sum(hover["roll"]) / len(hover["roll"]),
            "pitch": sum(hover["pitch"]) / len(hover["pitch"]),
        }
    # vibe
    if vibe_y:
        sv = sorted(vibe_y)
        out["vibe"] = (sum(vibe_y) / len(vibe_y), sv[int(len(sv) * 0.99)])
    # brake events
    att_t = [a[0] for a in att]
    def yaw_at(ts):
        i = max(0, min(len(att) - 1, bisect.bisect(att_t, ts)))
        return att[i][3]
    vx = []
    for ts, vn, ve in vel:
        y = math.radians(yaw_at(ts))
        vx.append((ts, vn * math.cos(y) + ve * math.sin(y)))
    def events(sign):
        evs = []; in_seg = False; seg_t0 = 0; pk = 0
        for ts, v in vx:
            vv = v * sign
            if vv > 2.0:
                if not in_seg: in_seg = True; seg_t0 = ts; pk = vv
                pk = max(pk, vv)
            elif in_seg and vv < 0.7:
                if ts - seg_t0 > 0.8: evs.append((seg_t0, ts, pk))
                in_seg = False; pk = 0
        return evs
    def wstats(t0, t1):
        ro = [(a[1], a[1] - a[4]) for a in att if t0 <= a[0] <= t1]
        rr = [r[1] for r in rate if t0 <= r[0] <= t1]
        if not ro: return None
        return (max(abs(r[0]) for r in ro), max(abs(r[1]) for r in ro),
                max(abs(r) for r in rr) if rr else 0)
    for label, sign in (("back", -1), ("fwd", 1)):
        rows = []
        for t0, t1, pk in events(sign):
            st = wstats(t1 - 0.5, t1 + 3.0)
            if st: rows.append((pk, st[0], st[1], st[2]))
        out[label] = rows
    return out

WATCH = ["ATC_ANG_RLL_P", "ATC_ANG_PIT_P", "ATC_RAT_RLL_P", "ATC_RAT_RLL_D",
         "ATC_RAT_PIT_P", "ATC_RAT_PIT_D", "PSC_POSXY_P", "PSC_VELXY_P",
         "PSC_VELXY_D", "LOIT_BRK_ACCEL", "LOIT_BRK_DELAY", "LOIT_SPEED",
         "INS_HNTCH_ENABLE", "INS_HNTCH_FREQ", "INS_ACCEL_FILTER",
         "INS_GYRO_FILTER", "ATC_THR_MIX_MAX", "MOT_THST_HOVER"]

prev_params = None
for path in sys.argv[1:]:
    name = path.replace("\\", "/").rsplit("/", 1)[-1]
    try:
        r = analyze(path)
    except Exception as e:
        print("\n=== %s : PARSE FAIL %s" % (name, e)); continue
    print("\n=== %s  (%.0fs, modes %s)" % (name, r["dur"], ",".join(r["modes"])))
    p = r["params"]
    if prev_params is not None:
        diff = {k: (prev_params.get(k), p.get(k)) for k in set(p) | set(prev_params)
                if abs((prev_params.get(k) or 0) - (p.get(k) or 0)) > 1e-6}
        shown = {k: v for k, v in diff.items() if not k.startswith(("STAT_", "SR0_", "SR1_", "SR2_", "COMPASS_OFS", "INS_ACCOFFS", "INS_GYROFFS", "GND_ABS_PRESS", "BARO"))}
        if shown:
            print("  changed vs prev: " + ", ".join(
                "%s %.4g->%.4g" % (k, v[0] if v[0] is not None else float('nan'),
                                   v[1] if v[1] is not None else float('nan'))
                for k, v in sorted(shown.items())[:12]))
        else:
            print("  changed vs prev: (none significant)")
    else:
        print("  key params: " + ", ".join("%s=%.4g" % (k, p[k]) for k in WATCH if k in p))
    prev_params = p
    if "hover" in r:
        h = r["hover"]
        print("  hover(n=%d): PWM FR%.0f RR%.0f RL%.0f FL%.0f | R-L %+.1fN  Re-Fr %+.1fN | roll %+.2f pitch %+.2f" %
              (h["n"], h["pwm"]["C1"], h["pwm"]["C2"], h["pwm"]["C3"], h["pwm"]["C4"],
               h["right_left_N"], h["rear_front_N"], h["roll"], h["pitch"]))
    if "vibe" in r:
        print("  vibeY: mean %.1f  p99 %.1f" % r["vibe"])
    for lbl in ("back", "fwd"):
        for pk, roll, rerr, rr in r[lbl]:
            print("  %s-brake: v=%.1f -> |roll| %.1f  err %.1f  rate %.0f dps" %
                  (lbl, pk, roll, rerr, rr))
