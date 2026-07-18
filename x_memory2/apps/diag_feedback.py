"""
diag_feedback.py — Feedback-path discriminator for the Z30 "마구 회전" problem.

Reads a flight_log produced by `SIL_App.exe ... gt` (the `gt` arg adds ground-truth
columns) and decides WHERE the attitude feedback breaks by comparing three signals:

  TRUE physics      : gt_roll/pitch/yaw_deg , gt_gyro_x/y/z   (from AirSim RPC)
  raw SHM (received) : gyro_x/y/z                             (what the app actually got)
  EKF estimate      : ekf_roll/pitch/yaw_deg                  (what the controller believes)

Verdict logic (evaluated over the airborne / takeoff-active segment):
  * TRUE attitude stays level  AND  EKF diverges          -> ESTIMATOR (EKF) bug
  * TRUE attitude really flips  AND  SHM gyro ~0           -> SENSOR/SHM bug (gyro not delivered)
  * TRUE flips  AND  SHM gyro tracks TRUE  AND EKF flips   -> genuine CONTROL instability (tune PID/mixer)
  * TRUE gyro ~0 everywhere but EKF ekf-attitude ramps     -> ESTIMATOR bug (phantom rate / bad gyro-bias)

Usage:
  python apps/diag_feedback.py [path_to_flight_log.csv]
  (default: d:/xlab/sim25/logs/silapp/flight_log_4000hz.csv)
"""
import csv
import os
import sys
import math

DEFAULT_LOG = "d:/xlab/sim25/logs/silapp/flight_log_4000hz.csv"

FLIP_DEG = 45.0      # |attitude| above this = "flipped/tumbling"
GYRO_ON = 0.10       # rad/s above this = "really rotating"
LEVEL_DEG = 10.0     # |attitude| below this = "level"


def _pf(s):
    # Robust float parse: MSVC prints NaN as "-nan(ind)", Inf as "inf" -> keep the row.
    try:
        return float(s)
    except (ValueError, TypeError):
        low = str(s).strip().lower()
        if "nan" in low:
            return float("nan")
        if "inf" in low:
            return float("-inf") if low.startswith("-") else float("inf")
        return float("nan")


def load(path):
    rows = []
    with open(path, "r") as f:
        reader = csv.reader(f)
        header = None
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            if header is None:
                header = [c.strip() for c in row]
                continue
            if len(row) < len(header):
                continue
            rows.append(dict(zip(header, [_pf(x) for x in row])))
    return header, rows


def col(rows, name):
    return [r.get(name, 0.0) for r in rows]


def _fin(xs):
    return [x for x in xs if x == x and abs(x) != float("inf")]  # drop NaN/Inf


def amax(xs):
    xs = _fin([abs(x) for x in xs])
    return max(xs, default=0.0)


def frac_nan(xs):
    return sum(1 for x in xs if not (x == x)) / len(xs) if xs else 0.0


def rms(xs):
    if not xs:
        return 0.0
    return math.sqrt(sum(x * x for x in xs) / len(xs))


def analyze(path):
    if not os.path.exists(path):
        print(f"Error: log not found: {path}")
        return
    header, rows = load(path)
    if not rows:
        print("No data rows.")
        return

    has_gt = "gt_roll_deg" in header
    if not has_gt:
        print("!! This log has NO ground-truth columns.")
        print("!! Re-run the controller with the `gt` argument, e.g.:")
        print("!!     SIL_App.exe 4000 z30 gt")
        print("!! then take off and re-run this script.")
        return

    # airborne / takeoff-active segment: pid_target > 0
    seg = [r for r in rows if r.get("pid_target", 0.0) > 0.0]
    if len(seg) < 50:
        print("Not enough takeoff-active samples (pid_target>0). Did the drone take off?")
        print(f"  total rows={len(rows)}  takeoff-active rows={len(seg)}")
        return

    # EKF attitude
    ekf_r, ekf_p, ekf_y = col(seg, "ekf_roll_deg"), col(seg, "ekf_pitch_deg"), col(seg, "ekf_yaw_deg")
    # TRUE attitude + rates
    gt_r, gt_p, gt_y = col(seg, "gt_roll_deg"), col(seg, "gt_pitch_deg"), col(seg, "gt_yaw_deg")
    gt_gx, gt_gy, gt_gz = col(seg, "gt_gyro_x"), col(seg, "gt_gyro_y"), col(seg, "gt_gyro_z")
    # raw SHM gyro (received by app)
    sh_gx, sh_gy, sh_gz = col(seg, "gyro_x"), col(seg, "gyro_y"), col(seg, "gyro_z")

    def att_mag(rs, ps):
        return [max(abs(a), abs(b)) for a, b in zip(rs, ps)]

    ekf_att = att_mag(ekf_r, ekf_p)
    gt_att = att_mag(gt_r, gt_p)
    gt_rate = [math.sqrt(a * a + b * b + c * c) for a, b, c in zip(gt_gx, gt_gy, gt_gz)]
    sh_rate = [math.sqrt(a * a + b * b + c * c) for a, b, c in zip(sh_gx, sh_gy, sh_gz)]

    # EKF NaN fraction (a diverged-to-NaN estimator is itself the smoking gun)
    ekf_nan_pct = 100.0 * frac_nan([a + b for a, b in zip(ekf_r, ekf_p)])

    # peak & typical (NaN/Inf-safe)
    ekf_att_max = amax(ekf_att)
    gt_att_max = amax(gt_att)
    gt_rate_max = max(_fin(gt_rate), default=0.0)
    sh_rate_max = max(_fin(sh_rate), default=0.0)

    # EKF-vs-TRUE attitude agreement (robust to the 50 Hz gt hold: attitude moves
    # slower than rate, so per-row |EKF-TRUE| is meaningful). This is the PRIMARY test:
    # if the EKF faithfully tracks the true attitude, the feedback path is trustworthy.
    att_err = _fin([abs(e - g) for e, g in zip(ekf_att, gt_att)])
    att_err.sort()
    ekf_att_med_err = att_err[len(att_err) // 2] if att_err else float("nan")
    # gyro peak ratio (peaks are alignment-robust, unlike per-row means)
    gyro_peak_ratio = (sh_rate_max / gt_rate_max) if gt_rate_max > 0.1 else float("nan")
    ekf_tracks_truth = (ekf_nan_pct < 1.0 and ekf_att_med_err == ekf_att_med_err
                        and ekf_att_med_err < 15.0 and ekf_att_max > 0.5 * gt_att_max)

    # How often SHM gyro tracks TRUE gyro when TRUE is really rotating:
    tracking_n = tracking_ok = 0
    for tr, sr in zip(gt_rate, sh_rate):
        if tr > GYRO_ON:
            tracking_n += 1
            if sr > 0.5 * tr:          # SHM sees at least half the true rate
                tracking_ok += 1
    track_pct = (100.0 * tracking_ok / tracking_n) if tracking_n else float("nan")

    # Rows where EKF says flipped (NaN counts as diverged)
    ekf_flip = [i for i, a in enumerate(ekf_att) if (not (a == a)) or a > FLIP_DEG]
    # of those, is TRUE also flipped?
    gt_flip_when_ekf = sum(1 for i in ekf_flip if gt_att[i] > FLIP_DEG)
    gt_flip_pct = (100.0 * gt_flip_when_ekf / len(ekf_flip)) if ekf_flip else float("nan")

    print("=" * 68)
    print("FEEDBACK-PATH DISCRIMINATOR")
    print("=" * 68)
    print(f"log: {path}")
    print(f"takeoff-active samples: {len(seg)}")
    print("-" * 68)
    print("PEAK |attitude| (deg)     TRUE(gt)   EKF(est)")
    print(f"                          {gt_att_max:8.1f}   {ekf_att_max:8.1f}")
    print(f"EKF attitude = NaN in     {ekf_nan_pct:.1f}% of takeoff-active samples")
    print("PEAK |gyro| (rad/s)       TRUE(gt)   SHM(recv)")
    print(f"                          {gt_rate_max:8.2f}   {sh_rate_max:8.2f}")
    print("-" * 68)
    print(f"EKF vs TRUE attitude: median |error| = {ekf_att_med_err:6.1f} deg  "
          f"(small => EKF tracks reality)")
    print(f"gyro peak ratio SHM/TRUE = {gyro_peak_ratio:5.2f}  (near 1 => gyro delivered)")
    print(f"EKF-flipped samples (|att|>{FLIP_DEG:.0f}): {len(ekf_flip)}")
    if ekf_flip:
        print(f"  ...of those, TRUE physics also flipped: {gt_flip_pct:.1f}%")
    if tracking_n:
        print(f"When TRUE is rotating (>{GYRO_ON} rad/s, {tracking_n} samples),")
        print(f"  SHM gyro tracks it (>=50%): {track_pct:.1f}%")
    else:
        print("TRUE physics never rotated appreciably during takeoff.")
    print("-" * 68)

    # ---- verdict (priority order; peaks + EKF-vs-TRUE agreement are alignment-robust) ----
    verdict = []
    if ekf_nan_pct > 5.0 and gt_att_max < FLIP_DEG:
        verdict.append(
            "==> ESTIMATOR (EKF) DIVERGED TO NaN.\n"
            "    The EKF blew up to NaN while the TRUE physics stayed level. The controller\n"
            "    then flies on garbage. Numerical instability in EST_EKF24, NOT a PID problem."
        )
    elif ekf_tracks_truth:
        verdict.append(
            "==> FEEDBACK IS TRUSTWORTHY -> GENUINE CONTROL INSTABILITY.\n"
            f"    The EKF tracks the true attitude (median err {ekf_att_med_err:.1f} deg) and the\n"
            f"    gyro peak is delivered (SHM/TRUE peak {gyro_peak_ratio:.2f}). The remaining flip\n"
            "    is a real controller problem: mixer desaturation + gain tuning (steps 2 & 3).\n"
            "    NOTE: low per-row gyro tracking% is mostly the 50Hz gt-hold artifact, not loss."
        )
    elif gt_rate_max > GYRO_ON and (gyro_peak_ratio != gyro_peak_ratio or gyro_peak_ratio < 0.30):
        verdict.append(
            "==> SENSOR / SHM STARVATION (feedback broken).\n"
            f"    Physics rotates to {gt_rate_max:.1f} rad/s but the SHM gyro the app receives\n"
            f"    peaks at only {sh_rate_max:.2f} (ratio {gyro_peak_ratio:.2f}). Inner rate loop and\n"
            "    EKF are starved of gyro. USUAL CAUSE: sim overload (RTF < 1) aliasing the\n"
            "    telemetry -> lower PhysicsLoopPeriod / raise RTF, OR fix the IMU SHM path.\n"
            "    PID tuning is meaningless until the gyro peak ratio is near 1."
        )
    elif ekf_att_max > FLIP_DEG and gt_att_max < FLIP_DEG:
        verdict.append(
            "==> ESTIMATOR (EKF) BUG.\n"
            "    The EKF reports a flip the TRUE physics does NOT show. The controller chases\n"
            "    a phantom attitude. Inspect EST_EKF24 before any PID tuning."
        )
    if not verdict:
        verdict.append(
            "==> INCONCLUSIVE / STABLE.\n"
            "    No flip and no clear divergence in this segment. If the drone still\n"
            "    misbehaves, capture a run that includes the failure and re-check."
        )
    print("VERDICT")
    for v in verdict:
        print(v)
    print("=" * 68)


if __name__ == "__main__":
    analyze(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG)
