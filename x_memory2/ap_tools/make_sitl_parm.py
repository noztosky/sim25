#!/usr/bin/env python3
"""Convert real_vehicle.param (from the .bin) into a SITL-loadable defaults file:
drop hardware-bound params that break SITL, add SITL fixups."""
SKIP_PREFIX = ("SERIAL", "CAN_", "COMPASS_DEV", "COMPASS_PRIO", "COMPASS_OFS", "COMPASS_DIA",
               "COMPASS_ODI", "COMPASS_MOT", "COMPASS_SCALE", "COMPASS_ORIENT", "AHRS_ORIENTATION",
               "INS_ACC_ID", "INS_GYR_ID", "INS_ACCOFFS", "INS_ACCSCAL", "INS_GYROFFS", "INS_GYR_CAL",
               "INS_ACC2", "INS_ACC3", "INS_GYR2", "INS_GYR3", "BRD_", "GPS_TYPE", "GPS_POS",
               "BARO_PROBE", "BATT_SERIAL", "BATT_MONITOR", "SPRAY", "RELAY", "SERVO",
               "RC_PROTOCOL", "FENCE_", "LOG_BACKEND", "SD_", "NTF_", "OSD", "VTX", "SCR_",
               "ARMING_CHECK", "SIM_")
FIXUPS = {
    # --- SITL fixups discovered by testing (each was a real blocker/crash) ---
    "AHRS_EKF_TYPE": "3", "EK3_ENABLE": "1",
    "FRAME_CLASS": "1", "FRAME_TYPE": "1",          # AirSim motor mapping (real=13 DJI-X, virtual here)
    "COMPASS_ENABLE": "1", "COMPASS_USE": "1", "EK3_SRC1_YAW": "1",  # real RTK GPS-yaw not simulatable
    "RC15_OPTION": "0",                              # real motor E-Stop switch (no RC15 in SITL)
    "TKOFF_RPM_MIN": "0",                            # ESC-RPM takeoff gate (no ESC telem in SITL)
    "INS_HNTCH_ENABLE": "0", "INS_HNTC2_ENABLE": "0",# ESC-telem notch -> div/0 FPE in SITL
    "BRD_SAFETY_DEFLT": "0", "DISARM_DELAY": "10",
    "INS_POS1_X": "0", "INS_POS1_Y": "0", "INS_POS1_Z": "0",  # lever arm disabled in sim IMU (A/B)
    "EK3_SRC1_POSZ": "1",  # real uses RTK GPS-Z; sim GPS-Z is laggy/noisy -> baro Z in SITL
}
src = "D:/xlab/sim25/x_memory2/build/real_vehicle.param"
dst = "D:/xlab/sim25/x_memory2/build/k20_real_sitl.parm"
kept = dropped = 0
with open(src) as f, open(dst, "w") as o:
    o.write("# EFT K20 real-vehicle parameters (from 2026-06-08 AutoTune .bin), SITL-safe subset\n")
    for line in f:
        line = line.strip()
        if not line or "," not in line:
            continue
        name, val = line.split(",", 1)
        if any(name.startswith(p) for p in SKIP_PREFIX):
            dropped += 1
            continue
        if name in FIXUPS:
            val = FIXUPS[name]
        o.write("%s %s\n" % (name, val))
        kept += 1
    for k, v in FIXUPS.items():
        o.write("%s %s\n" % (k, v))
print("kept=%d dropped=%d -> %s" % (kept, dropped, dst))
