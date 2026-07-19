#!/usr/bin/env python3
"""Hover motor-split check: takeoff, hover N s, average SERVO_OUTPUT_RAW,
convert to thrust with the sim's quadratic curve and compare the left-right /
rear-front thrust splits against the real-log targets (27.2 N right, 5.7 N rear).
AirSim FRAME_TYPE=1 X: M1 FR, M2 RL, M3 FL, M4 RR.
Usage: python ap_hover_check.py [hover_sec]
"""
import sys, time
from pymavlink import mavutil

HOVER_S = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
TMAX = 259.9

m = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
m.wait_heartbeat(timeout=60)
m.mav.request_data_stream_send(m.target_system, m.target_component,
                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
t0 = time.time()
while time.time() - t0 < 120:
    msg = m.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=5)
    if msg and msg.flags & 16:
        break
# wait disarm from any previous flight
t0 = time.time()
while time.time() - t0 < 90:
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        break
m.set_mode_apm("GUIDED"); time.sleep(1)
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
m.recv_match(type="COMMAND_ACK", blocking=True, timeout=4)
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
t0 = time.time()
while time.time() - t0 < 45:
    g = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    if g and g.relative_alt / 1000.0 > 9.5:
        break
else:
    sys.exit("takeoff stalled")
time.sleep(5)  # settle

acc = {1: [], 2: [], 3: [], 4: []}
t0 = time.time()
while time.time() - t0 < HOVER_S:
    s = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=3)
    if s:
        acc[1].append(s.servo1_raw); acc[2].append(s.servo2_raw)
        acc[3].append(s.servo3_raw); acc[4].append(s.servo4_raw)
m.set_mode_apm("LAND")

mean = {k: sum(v) / len(v) for k, v in acc.items()}
thr = {k: ((mean[k] - 1000) / 1000.0) ** 2 * TMAX for k in mean}
# M1 FR, M2 RL, M3 FL, M4 RR
right = thr[1] + thr[4]; left = thr[2] + thr[3]
rear = thr[2] + thr[4]; front = thr[1] + thr[3]
print("hover PWM  : M1_FR %.0f  M2_RL %.0f  M3_FL %.0f  M4_RR %.0f  (n=%d)" %
      (mean[1], mean[2], mean[3], mean[4], len(acc[1])))
print("thrusts [N]: FR %.1f  RL %.1f  FL %.1f  RR %.1f  total %.1f" %
      (thr[1], thr[2], thr[3], thr[4], sum(thr.values())))
print("split right-left : %+.1f N   [real target +27.2]" % (right - left))
print("split rear-front : %+.1f N   [real target  +5.7]" % (rear - front))
