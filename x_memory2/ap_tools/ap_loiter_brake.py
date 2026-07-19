#!/usr/bin/env python3
"""Loiter backward->stop brake test (baseline for the real K20 'roll wobble on brake').

Sequence: GUIDED takeoff 10m -> LOITER -> RC pitch stick back N sec -> release
-> record attitude/rates/velocity through the brake -> LAND.
Writes CSV to logs/silapp/ap_brake_<tag>.csv ; SITL .BIN has the full-rate truth.

Usage: python ap_loiter_brake.py [tag] [push_sec] [roll_pwm] [pitch_pwm]
  roll_pwm : PUSH-phase roll stick (default 1500 = none; 1100 = full left
  for a diagonal back-left run matching the real log event at t=951).
  pitch_pwm: PUSH-phase pitch stick (default 1900 = full back; 1100 = full forward).
"""
import sys, time, csv, os
from pymavlink import mavutil

TAG = sys.argv[1] if len(sys.argv) > 1 else "baseline"
PUSH_SEC = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
ROLL_PWM = int(sys.argv[3]) if len(sys.argv) > 3 else 1500
PITCH_PWM = int(sys.argv[4]) if len(sys.argv) > 4 else 1900
ALT = 10.0
CSV_PATH = os.path.join(os.path.dirname(__file__), "..", "..", "logs", "silapp",
                        "ap_brake_%s.csv" % TAG)
NO_OVR = 65535  # leave channel untouched

m = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
m.wait_heartbeat(timeout=60)
print("heartbeat ok (sys %d)" % m.target_system)
m.mav.request_data_stream_send(m.target_system, m.target_component,
                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

# --- wait EKF position ---
t0 = time.time()
while time.time() - t0 < 120:
    msg = m.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=5)
    if msg and msg.flags & 16:
        print("EKF pos OK (%.0fs)" % (time.time() - t0)); break
else:
    sys.exit("EKF never got position")

def rc(ch1=NO_OVR, ch2=NO_OVR, ch3=NO_OVR, ch4=NO_OVR):
    m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                    ch1, ch2, ch3, ch4,
                                    NO_OVR, NO_OVR, NO_OVR, NO_OVR)

def wait_ack(name=""):
    a = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=4)
    if a: print("  ack %s: %d" % (name, a.result))

# --- wait for any previous flight to finish landing/disarm ---
t0 = time.time()
while time.time() - t0 < 90:
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        break
else:
    print("warning: still armed after 90s, proceeding anyway")

# --- takeoff ---
m.set_mode_apm("GUIDED"); time.sleep(1)
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                        1, 0, 0, 0, 0, 0, 0)
wait_ack("arm")
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                        0, 0, 0, 0, 0, 0, ALT)
wait_ack("takeoff")
t0 = time.time()
while time.time() - t0 < 45:
    g = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    if g and g.relative_alt / 1000.0 > ALT * 0.95:
        print("at altitude %.2f m (%.0fs)" % (g.relative_alt / 1000.0, time.time() - t0))
        break
else:
    sys.exit("takeoff stalled")
time.sleep(4)  # settle

# --- LOITER with neutral sticks ---
rc(1500, 1500, 1500, 1500)
m.set_mode_apm("LOITER")
t0 = time.time()
mode = "?"
while time.time() - t0 < 5:
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    if hb:
        mode = mavutil.mode_string_v10(hb)
        if mode == "LOITER":
            break
    rc(1500, 1500, 1500, 1500)
print("mode now:", mode)
if mode != "LOITER":
    sys.exit("mode switch to LOITER failed")
t0 = time.time()
while time.time() - t0 < 3:
    rc(1500, 1500, 1500, 1500); time.sleep(0.1)

# --- record while: push back PUSH_SEC, release, watch brake 12s ---
rows = []
vibes = []
last_nav = None
phase_t0 = time.time()
phase = "push"
print("PUSH back (%.1fs)..." % PUSH_SEC)
last_att = last_pos = None
while True:
    t = time.time() - phase_t0
    if phase == "push":
        rc(ROLL_PWM, PITCH_PWM, 1500, 1500)
        if t >= PUSH_SEC:
            phase = "brake"; print("RELEASE -> brake")
    elif phase == "brake":
        rc(1500, 1500, 1500, 1500)
        if t >= PUSH_SEC + 12:
            break
    msg = m.recv_match(type=["ATTITUDE", "LOCAL_POSITION_NED", "VIBRATION",
                             "NAV_CONTROLLER_OUTPUT"], blocking=True, timeout=2)
    if msg is None:
        print("no data at t=%.1f" % t); continue
    mt = msg.get_type()
    if mt == "ATTITUDE":
        last_att = msg
    elif mt == "VIBRATION":
        vibes.append((msg.vibration_x, msg.vibration_y, msg.vibration_z))
        continue
    elif mt == "NAV_CONTROLLER_OUTPUT":
        last_nav = msg
        continue
    else:
        last_pos = msg
    if last_att and last_pos:
        yaw = last_att.yaw
        import math as _m
        vyb = -last_pos.vx * _m.sin(yaw) + last_pos.vy * _m.cos(yaw)
        nav_roll = last_nav.nav_roll if last_nav else 0.0
        rows.append([round(t, 3), phase,
                     round(last_att.roll * 57.2958, 3), round(last_att.pitch * 57.2958, 3),
                     round(last_att.rollspeed * 57.2958, 2), round(last_att.pitchspeed * 57.2958, 2),
                     round(last_pos.vx, 3), round(last_pos.vy, 3), round(last_pos.z, 3),
                     round(vyb, 3), round(nav_roll, 2),
                     round(last_att.roll * 57.2958 - nav_roll, 2)])

# --- land ---
rc()  # clear overrides
m.set_mode_apm("LAND")
print("landing...")

os.makedirs(os.path.dirname(CSV_PATH), exist_ok=True)
with open(CSV_PATH, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "phase", "roll_deg", "pitch_deg", "rollrate_dps", "pitchrate_dps",
                "vn", "ve", "z", "vy_body", "nav_roll", "roll_err"])
    w.writerows(rows)
print("wrote %d rows -> %s" % (len(rows), CSV_PATH))

# quick verdict
br = [r for r in rows if r[1] == "brake"]
pu = [r for r in rows if r[1] == "push"]
if pu:
    import math
    n5 = [r for r in pu if r[0] > PUSH_SEC * 0.6]
    if n5:
        print("push late-phase: mean pitch %.1f deg  mean |v| %.2f m/s" %
              (sum(r[3] for r in n5) / len(n5),
               sum(math.hypot(r[6], r[7]) for r in n5) / len(n5)))
if br and pu:
    import math
    peak_roll = max(abs(r[2]) for r in br)
    peak_rr = max(abs(r[4]) for r in br)
    peak_pitch = max(abs(r[3]) for r in br)
    vmax = max(math.hypot(r[6], r[7]) for r in pu)
    print("push  : vmax=%.2f m/s  pitch range [%.1f, %.1f]  vy_body range [%.2f, %.2f]" %
          (vmax, min(r[3] for r in pu), max(r[3] for r in pu),
           min(r[9] for r in pu), max(r[9] for r in pu)))
    print("brake : |roll|peak=%.2f deg  |rollrate|peak=%.1f dps  |pitch|peak=%.1f deg" %
          (peak_roll, peak_rr, peak_pitch))
    print("brake : |roll_err|peak=%.2f deg  vy_body range [%.2f, %.2f]  [real: err 4.6, vy -2.2->+1.3]" %
          (max(abs(r[11]) for r in br), min(r[9] for r in br), max(r[9] for r in br)))
if vibes:
    n = len(vibes)
    print("vibe  : mean(x,y,z)=(%.1f, %.1f, %.1f)  peak(x,y,z)=(%.1f, %.1f, %.1f)  [real Y: 7.2/22.3]" %
          (sum(v[0] for v in vibes)/n, sum(v[1] for v in vibes)/n, sum(v[2] for v in vibes)/n,
           max(v[0] for v in vibes), max(v[1] for v in vibes), max(v[2] for v in vibes)))
