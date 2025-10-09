#pragma once

// External x_memory integration macros (ring buffer)
// Default: use local copy at D:\open\airsim\x_memory\fc_ring.h
#ifndef XLAB_XMEMORY_HEADER
#define XLAB_XMEMORY_HEADER "D:\\open\\airsim\\x_memory\\fc_ring.h"
#endif

// Use fc_ring C++ class API (SPSC ring)
#define XLAB_XMEMORY_TYPE           fc_ring
#define XLAB_XMEMORY_OPEN(obj, path)            (obj.server_create(path))
#define XLAB_XMEMORY_OPEN_READER(obj, path)     (obj.client_connect(path))

// High-level IMU write mapping (Euler in degrees accepted by client UI)
#define XLAB_XMEMORY_WRITE_IMU_EULER(impl, r, p, y, ts_ns, hz) do { \
    ImuData d{}; \
    d.roll = (r); d.pitch = (p); d.yaw = (y); \
    d.timestamp = (ts_ns); d.frequency = (hz); d.is_valid = true; \
    impl.publish_imu(d); \
} while(0)

// PWM read mapping (single-slot compatibility API provided by fc_ring)
#define XLAB_XMEMORY_TRY_GET_PWM(impl, out_pwm_struct)   (impl.try_get_pwm(out_pwm_struct))

// Baro write mapping (altitude in meters)
#define XLAB_XMEMORY_WRITE_BARO(impl, altitude_m, ts_ns, hz) do { \
    BaroData b{}; \
    b.altitude = (altitude_m); b.timestamp = (ts_ns); b.frequency = (hz); b.is_valid = true; \
    impl.publish_baro(b); \
} while(0)


