#!/usr/bin/env python3
"""Parse the real vehicle's .bin dataflash log (AutoTune flight):
extract full parameters -> real_vehicle.param, firmware/frame info,
vibration levels, motor-output asymmetry, autotune results."""
import sys
from collections import defaultdict
from pymavlink import DFReader

path = sys.argv[1] if len(sys.argv) > 1 else "real_autotune.bin"
log = DFReader.DFReader_binary(path)

params = {}
msgs = []
vibe = defaultdict(list)
rcou = defaultdict(list)
att_err_r, att_err_p = [], []
atun = []
modes = []
n = 0
while True:
    m = log.recv_msg()
    if m is None:
        break
    n += 1
    t = m.get_type()
    if t == "PARM":
        params[m.Name] = m.Value
    elif t == "MSG":
        s = str(m.Message)
        if len(msgs) < 25:
            msgs.append(s)
    elif t == "VIBE":
        vibe["x"].append(m.VibeX); vibe["y"].append(m.VibeY); vibe["z"].append(m.VibeZ)
        c = getattr(m, "Clip", None)
        if c is None:
            c = getattr(m, "Clip0", 0) + getattr(m, "Clip1", 0) + getattr(m, "Clip2", 0)
        vibe["c0"].append(c); vibe["c1"].append(0); vibe["c2"].append(0)
    elif t == "RCOU":
        rcou[1].append(m.C1); rcou[2].append(m.C2); rcou[3].append(m.C3); rcou[4].append(m.C4)
    elif t == "ATT":
        att_err_r.append(abs(m.DesRoll - m.Roll))
        att_err_p.append(abs(m.DesPitch - m.Pitch))
    elif t == "ATUN":
        atun.append(m)
    elif t == "MODE":
        modes.append(str(m.Mode))

print("total messages: %d" % n)
print("\n=== FIRMWARE / FRAME ===")
for s in msgs[:10]:
    print(" ", s)
print("\n=== MODES flown ===", sorted(set(modes)))

print("\n=== PARAMS: %d extracted ===" % len(params))
out = "D:/xlab/sim25/x_memory2/build/real_vehicle.param"
with open(out, "w") as f:
    for k in sorted(params):
        f.write("%s,%s\n" % (k, repr(params[k]).rstrip("0").rstrip(".") if isinstance(params[k], float) else params[k]))
print("saved ->", out)

KEY = ["FRAME_CLASS", "FRAME_TYPE", "MOT_THST_HOVER", "MOT_THST_EXPO", "MOT_SPIN_MIN",
       "ATC_RAT_PIT_P", "ATC_RAT_PIT_I", "ATC_RAT_PIT_D", "ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D",
       "ATC_ANG_PIT_P", "ATC_ANG_RLL_P", "ATC_ANG_YAW_P", "ATC_INPUT_TC", "ATC_ACCEL_P_MAX", "ATC_ACCEL_R_MAX",
       "INS_GYRO_FILTER", "INS_ACCEL_FILTER", "INS_POS1_X", "INS_POS_X",
       "INS_HNTCH_ENABLE", "INS_HNTCH_FREQ", "INS_HNTCH_REF", "INS_HNTCH_MODE",
       "LOIT_SPEED", "LOIT_ANG_MAX", "LOIT_BRK_ACCEL", "LOIT_BRK_DELAY", "LOIT_BRK_JERK", "LOIT_ACC_MAX",
       "PSC_POSZ_P", "PSC_VELZ_P", "PILOT_SPEED_UP", "PILOT_SPEED_DN",
       "PSC_POSXY_P", "PSC_VELXY_P", "PSC_VELXY_I", "PSC_VELXY_D",
       "SCHED_LOOP_RATE", "GPS_RATE_MS", "EK3_ENABLE", "AHRS_EKF_TYPE",
       "BATT_CAPACITY", "AUTOTUNE_AXES", "AUTOTUNE_AGGR"]
print("\n=== KEY PARAMS ===")
for k in KEY:
    if k in params:
        print("  %-18s = %s" % (k, params[k]))

if vibe["x"]:
    import statistics as st
    print("\n=== VIBRATION (m/s^2) ===")
    for ax in ("x", "y", "z"):
        print("  Vibe%s: mean=%.1f max=%.1f" % (ax.upper(), st.mean(vibe[ax]), max(vibe[ax])))
    print("  Clips: %d / %d / %d" % (max(vibe["c0"]), max(vibe["c1"]), max(vibe["c2"])))

if rcou[1]:
    import statistics as st
    print("\n=== MOTOR OUTPUTS (PWM, mean) ===")
    means = {i: st.mean(rcou[i]) for i in (1, 2, 3, 4)}
    for i in (1, 2, 3, 4):
        print("  C%d: %.0f" % (i, means[i]))
    print("  (QuadX: C1=FR? per FRAME; front-vs-rear asymmetry check)")

if att_err_r:
    import statistics as st
    print("\n=== ATT TRACKING |Des-Act| (deg) ===")
    print("  roll : mean=%.2f max=%.1f" % (st.mean(att_err_r), max(att_err_r)))
    print("  pitch: mean=%.2f max=%.1f" % (st.mean(att_err_p), max(att_err_p)))

print("\n=== AUTOTUNE messages: %d ===" % len(atun))
for m in atun[-6:]:
    print(" ", m)
