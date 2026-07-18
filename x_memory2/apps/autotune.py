"""
autotune.py — step-response PID auto-tuner for the EFT Z30 (SIL_App), AltHold mode.

Tunes the way a real FC is tuned: command attitude STEPS and score the response
(overshoot, settling, tracking error), not just "did it hover". Each trial:
  restart SIL_App -> mode althold -> takeoff -> stabilize -> pitch step -> roll step
  -> read flight_log -> classic step-response metrics -> cost.
A Nelder-Mead optimizer searches the gain vector; every trial is appended to
tuning_log.csv. Robust to the UE sim crashing (relaunches sblocks and continues).

Prereqs: sim reachable (sblocks.bat 1000). This script owns SIL_App + can relaunch UE.

Usage:
  cd d:/xlab/sim25/x_memory2
  python apps/autotune.py                       # attitude-precision phase, ~35 evals
  python apps/autotune.py --phase altitude       # altitude-hold phase
  python apps/autotune.py --evals 50 --repeat 2  # more evals, 2 reps/point (noise-robust)
"""
import argparse
import csv
import math
import os
import socket
import subprocess
import time

import numpy as np
from scipy.optimize import minimize

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
BUILD = os.path.join(HERE, "..", "build")
PARAMS_PATH = os.path.join(BUILD, "pid_params_z30.txt")
LOG_PATH = "d:/xlab/sim25/logs/silapp/flight_log_1000hz.csv"
SIL_EXE = os.path.join(BUILD, "SIL_App.exe")
SBLOCKS = os.path.join(ROOT, "sblocks.bat")
TUNING_LOG = os.path.join(BUILD, "tuning_log.csv")
UDP = ("127.0.0.1", 5005)
TARGET_ALT = 5.0
STEP_DEG = 10.0   # attitude step magnitude

# Parameter phases: (key, lo, hi, x0). Attitude precision first (real-FC order).
PHASES = {
    "attitude": [
        ("rate_p",     0.04, 0.30, 0.0978),
        ("rate_i",     0.00, 0.20, 0.10),
        ("rate_d",     0.00, 0.10, 0.0289),
        ("again_rp",   1.50, 6.00, 3.089),
        ("pitch_trim", -0.05, 0.20, 0.0128),
    ],
    "altitude": [
        ("alt_p",    0.03, 0.35, 0.1236),
        ("alt_i",    0.00, 0.15, 0.03),
        ("alt_d",    0.00, 0.35, 0.1442),
        ("alt_max",  0.05, 0.40, 0.10),
    ],
}


def send(msg):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.sendto(msg.encode(), UDP)
    finally:
        s.close()


def ps(cmd):
    return subprocess.run(["powershell", "-NoProfile", "-Command", cmd],
                          capture_output=True, text=True)


def rpc_up():
    r = ps("if(Test-NetConnection 127.0.0.1 -Port 41451 -WarningAction SilentlyContinue "
           "-InformationLevel Quiet){'Y'}else{'N'}")
    return "Y" in r.stdout


def kill_sil():
    subprocess.run(["taskkill", "/F", "/IM", "SIL_App.exe"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def ensure_sim():
    """Relaunch the UE sim if its RPC is down (crash recovery). One instance only."""
    if rpc_up():
        return True
    print("  [sim] RPC down -> relaunching UE ...", flush=True)
    subprocess.run(["taskkill", "/F", "/IM", "UE4Editor.exe"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(5)
    subprocess.Popen(["cmd", "/c", SBLOCKS, "1000"], cwd=ROOT,
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(60):
        time.sleep(3)
        if rpc_up():
            time.sleep(22)   # level load + vehicle spawn
            print("  [sim] back up.", flush=True)
            return True
    print("  [sim] FAILED to come back up.", flush=True)
    return False


def write_params(overrides):
    ov = dict(overrides)
    if "again_rp" in ov:
        v = ov.pop("again_rp"); ov["again_roll"] = v; ov["again_pitch"] = v
    with open(PARAMS_PATH, "r", encoding="utf-8") as f:
        lines = f.readlines()
    seen, out = set(), []
    for line in lines:
        raw = line.rstrip("\n"); code = raw.split("#", 1)[0]; comment = raw[len(code):]
        if "=" in code:
            key = code.split("=", 1)[0].strip()
            if key in ov:
                out.append(f"{key} = {ov[key]:.6g}{('  ' + comment) if comment else ''}\n")
                seen.add(key); continue
        out.append(line)
    for k, v in ov.items():
        if k not in seen:
            out.append(f"{k} = {v:.6g}\n")
    with open(PARAMS_PATH, "w", encoding="utf-8") as f:
        f.writelines(out)


def read_new_rows(offset):
    header = None
    with open(LOG_PATH, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("#"):
                continue
            header = [c.strip() for c in line.strip().split(",")]
            break
    rows = []
    with open(LOG_PATH, "r", encoding="utf-8", errors="replace") as f:
        f.seek(offset)
        for line in f:
            if not line.strip() or line.startswith("#"):
                continue
            parts = line.strip().split(",")
            if len(parts) < len(header):
                continue

            def pf(x):
                try:
                    return float(x)
                except ValueError:
                    return float("nan")

            rows.append(dict(zip(header, [pf(p) for p in parts])))
    return rows


def _gp(d):
    v = d.get("gt_pitch_deg", d.get("ekf_pitch_deg", 0.0))
    return 0.0 if v != v else v


def _gr(d):
    v = d.get("gt_roll_deg", d.get("ekf_roll_deg", 0.0))
    return 0.0 if v != v else v


def step_metrics(rows, axis_key, getter):
    """Tracking RMS + overshoot for the window where the commanded axis is stepped."""
    step = [d for d in rows if d.get(axis_key, 0.0) > STEP_DEG * 0.5]
    if len(step) < 15:
        return None
    hold = step[int(0.4 * len(step)):]            # skip the rise transient
    err = [getter(d) - STEP_DEG for d in hold]
    rms = math.sqrt(sum(e * e for e in err) / len(err))
    overshoot = max(0.0, max(getter(d) for d in step) - STEP_DEG)
    return rms, overshoot


def evaluate(rows):
    seg = [d for d in rows if d.get("pid_target", 0.0) > 0.0]
    if len(seg) < 300:
        return None
    peak = max(max(abs(_gp(d)), abs(_gr(d))) for d in seg)
    max_alt = max((d.get("ekf_alt", 0.0) for d in seg
                   if d.get("ekf_alt", 0.0) == d.get("ekf_alt", 0.0)), default=0.0)
    if peak > 60.0 or max_alt < 3.0:
        return {"crash": True, "peak": peak, "max_alt": max_alt}

    pm = step_metrics(seg, "tgt_pitch_deg", _gp)
    rm = step_metrics(seg, "tgt_roll_deg", _gr)
    # hover noise: level rows (no commanded tilt)
    lvl = [d for d in seg if abs(d.get("tgt_pitch_deg", 0)) < 1 and abs(d.get("tgt_roll_deg", 0)) < 1]
    if len(lvl) > 40:
        mid = lvl[len(lvl) // 3: 2 * len(lvl) // 3]
        att = [max(abs(_gp(d)), abs(_gr(d))) for d in mid]
        hover_noise = math.sqrt(sum(a * a for a in att) / len(att)) if att else 0.0
    else:
        hover_noise = 0.0
    alt_dev = max(abs(d.get("ekf_alt", 5.0) - TARGET_ALT) for d in seg)

    if pm is None and rm is None:
        return {"crash": True, "peak": peak, "max_alt": max_alt}
    track = np.mean([m[0] for m in (pm, rm) if m])
    over = np.mean([m[1] for m in (pm, rm) if m])
    return {"crash": False, "track": float(track), "overshoot": float(over),
            "hover_noise": hover_noise, "alt_dev": alt_dev, "peak": peak}


# precision-focused cost weights
W_TRACK = 1.0     # steady-state tracking error of the step (deg)
W_OVER = 0.6      # step overshoot (deg)
W_NOISE = 2.0     # hover attitude noise (deg RMS)
W_ALTDEV = 1.5    # altitude deviation during maneuver (m)


def cost_of(m):
    if m is None:
        return 500.0
    if m.get("crash"):
        return 200.0 + m.get("peak", 90.0) + max(0.0, 5.0 - m.get("max_alt", 0.0)) * 10.0
    return (W_TRACK * m["track"] + W_OVER * m["overshoot"]
            + W_NOISE * m["hover_noise"] + W_ALTDEV * m["alt_dev"])


class Tuner:
    def __init__(self, tuned, repeat):
        self.tuned = tuned
        self.repeat = repeat
        self.trial = 0
        self.best_cost = float("inf")
        self.best_x = None
        keys = [k for k, *_ in tuned]
        self.header = ["trial", "wall_s", "cost"] + keys + \
            ["track", "overshoot", "hover_noise", "alt_dev", "peak", "crash"]
        with open(TUNING_LOG, "w", newline="") as f:
            csv.writer(f).writerow(self.header)
        self.t0 = time.time()

    def one_flight(self, ov):
        if not ensure_sim():
            return None
        write_params(ov)
        kill_sil()
        sil = subprocess.Popen([SIL_EXE, "1000", "z30", "gt"], cwd=BUILD,
                               stdout=open(os.path.join(BUILD, "autotune_sil.log"), "w"),
                               stderr=subprocess.STDOUT)
        try:
            time.sleep(7.0)                       # connect + startup reset + EKF converge
            send("mode althold")
            time.sleep(0.3)
            send(f"a {TARGET_ALT:.1f} 10")
            time.sleep(5.0)                        # climb + stabilize
            offset = os.path.getsize(LOG_PATH)
            # pitch step + return
            for _ in range(16):
                send(f"c 0 {STEP_DEG:.0f} 0 0.3"); time.sleep(0.1)
            for _ in range(16):
                send("c 0 0 0 0.3"); time.sleep(0.1)
            # roll step + return
            for _ in range(16):
                send(f"c {STEP_DEG:.0f} 0 0 0.3"); time.sleep(0.1)
            for _ in range(16):
                send("c 0 0 0 0.3"); time.sleep(0.1)
            time.sleep(0.5)
            return evaluate(read_new_rows(offset))
        finally:
            sil.terminate()
            try:
                sil.wait(timeout=3)
            except Exception:
                sil.kill()
            time.sleep(1.5)

    def objective(self, x):
        x = np.array([min(hi, max(lo, xi)) for xi, (_, lo, hi, _) in zip(x, self.tuned)])
        self.trial += 1
        ov = {k: float(v) for (k, *_), v in zip(self.tuned, x)}
        # repeat for noise robustness -> use WORST (most conservative) cost
        results = [self.one_flight(ov) for _ in range(self.repeat)]
        costs = [cost_of(m) for m in results]
        c = max(costs)
        m = results[int(np.argmax(costs))]

        if c < self.best_cost:
            self.best_cost = c
            self.best_x = x.copy()

        row = [self.trial, round(time.time() - self.t0, 1), round(c, 4)]
        row += [round(float(v), 6) for v in x]
        if m is None or m.get("crash"):
            row += ["", "", "", "", round(m.get("peak", 0), 1) if m else "", 1 if m else ""]
        else:
            row += [round(m["track"], 3), round(m["overshoot"], 2), round(m["hover_noise"], 3),
                    round(m["alt_dev"], 2), round(m["peak"], 1), 0]
        with open(TUNING_LOG, "a", newline="") as f:
            csv.writer(f).writerow(row)

        tag = "CRASH" if (m is None or m.get("crash")) else "ok  "
        parts = "  ".join(f"{k}={v:.4g}" for (k, *_), v in zip(self.tuned, x))
        extra = "" if (m is None or m.get("crash")) else \
            f" track={m['track']:.2f} over={m['overshoot']:.1f} noise={m['hover_noise']:.2f} altdev={m['alt_dev']:.2f}"
        print(f"[{self.trial:2d}] {tag} cost={c:7.3f} | {parts}{extra}  (best={self.best_cost:.3f})", flush=True)
        return c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase", choices=["attitude", "altitude"], default="attitude")
    ap.add_argument("--evals", type=int, default=35)
    ap.add_argument("--repeat", type=int, default=1)
    args = ap.parse_args()

    tuned = PHASES[args.phase]
    tuner = Tuner(tuned, args.repeat)
    x0 = np.array([x for *_, x in tuned], dtype=float)
    bounds = [(lo, hi) for _, lo, hi, _ in tuned]

    # Wide initial simplex so Nelder-Mead actually EXPLORES (default step ~5% of x0 is
    # far too small for these small-magnitude gains -> it stalls at the start point).
    steps = np.array([0.4 * (hi - lo) for _, lo, hi, _ in tuned])
    simplex = [x0.copy()]
    for i in range(len(x0)):
        v = x0.copy()
        v[i] = min(bounds[i][1], max(bounds[i][0], x0[i] + steps[i]))
        simplex.append(v)
    simplex = np.array(simplex)

    print(f"[autotune] phase={args.phase}  AltHold step-response tuning  "
          f"({len(tuned)}D, up to {args.evals} evals x{args.repeat} rep, wide simplex)", flush=True)
    try:
        minimize(tuner.objective, x0, method="Nelder-Mead", bounds=bounds,
                 options=dict(maxfev=args.evals, xatol=1e-3, fatol=1e-2, initial_simplex=simplex))
    except KeyboardInterrupt:
        print("\n[autotune] interrupted", flush=True)

    print("\n" + "=" * 60)
    if tuner.best_x is not None:
        best = {k: float(v) for (k, *_), v in zip(tuned, tuner.best_x)}
        print(f"BEST cost={tuner.best_cost:.3f}")
        for (k, *_), v in zip(tuned, tuner.best_x):
            print(f"  {k:12s} = {v:.5g}")
        write_params(best)
        print(f"\n-> wrote best {args.phase} gains to {PARAMS_PATH}")
    else:
        print("No successful trial.")
    print(f"-> history: {TUNING_LOG}")
    print("=" * 60)
    kill_sil()


if __name__ == "__main__":
    main()
