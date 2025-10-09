#pragma once

// External x_memory integration macros (pure ASCII)
// Default: use local copy at D:\open\airsim\x_memory\fc_memory.h
#ifndef XLAB_XMEMORY_HEADER
#define XLAB_XMEMORY_HEADER "D:\\open\\airsim\\x_memory\\fc_memory.h"
#endif

// Use fc_memory C++ class API
#define XLAB_XMEMORY_TYPE           fc_memory
#define XLAB_XMEMORY_OPEN(obj, path)            (obj.server_create(path))
#define XLAB_XMEMORY_OPEN_READER(obj, path)     (obj.client_connect(path))

// High-level IMU write mapping (Euler angles in radians)
#define XLAB_XMEMORY_WRITE_IMU_EULER(impl, r, p, y, ts_ns, hz) do { \
    ImuData d{}; \
    d.roll = (r); d.pitch = (p); d.yaw = (y); \
    d.timestamp = (ts_ns); d.frequency = (hz); d.is_valid = true; \
    impl.publish_imu(d); \
} while(0)

// PWM read mapping
#define XLAB_XMEMORY_TRY_GET_PWM(impl, out_pwm_struct)   (impl.try_get_pwm(out_pwm_struct))


