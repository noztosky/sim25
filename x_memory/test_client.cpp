#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <cstdint>
#include <sstream>
#include <cmath>

#include "x_xsim.h"
#include "lib/LogHelper.hpp"
#include "lib/XSimIo.hpp"
#include "lib/RotorWriter.hpp"
#include "lib/AttitudeUtils.hpp"
#include "lib/PidController.hpp"

// Scenario selection
#define SCENARIO_CARDINAL_TILT     1
#define SCENARIO_ALL_1600          2
#define SCENARIO_TAKEOFF_HOVER_BIAS 3

#ifndef TEST_SCENARIO
#define TEST_SCENARIO           SCENARIO_TAKEOFF_HOVER_BIAS
#endif

static const int PWM_TX_HZ = 400;

// Scenario: forward → right → back → left tilt sequence once
static void scenario_cardinal_tilt(XSimIo& io, LogHelper& logger)
{
	// Timing
	auto period = std::chrono::microseconds(1000000 / PWM_TX_HZ);
	auto start = std::chrono::high_resolution_clock::now();
	auto next_tp = start + period;
	auto last_log = start;
	const auto LOG_INTERVAL = std::chrono::milliseconds(100);

	// Sequence config
	const int TEST_PWM_LOW = 1599;  // LOW thrust
	const int TEST_PWM_HIGH = 1600; // HIGH thrust
	const int HOLD_MS = 1000;       // hold per direction
	const int GAP_MS = 1000;        // gap between directions
	const int SLOT_LEN_MS = HOLD_MS + GAP_MS; // 2000ms per slot
	// slots: 0:F,1:gap,2:R,3:gap,4:B,5:gap,6:L,7:gap

	RotorWriter writer(io.handle());
	// Telemetry tracking for attitude logging
	uint32_t last_seq = 0;
	XSimTelemetry last_telem{}; bool have_telem = false;
	bool running = true;
	while (running) {
		auto now = std::chrono::high_resolution_clock::now();
		if (now >= next_tp) {
			int elapsed_ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
			int slot = elapsed_ms / SLOT_LEN_MS;
			int in_slot = elapsed_ms % SLOT_LEN_MS;
			bool in_hold = (in_slot < HOLD_MS);

			uint16_t r1, r2, r3, r4; // FR, RL, FL, RR
			// default: hover-ish baseline
			r1 = r2 = r3 = r4 = 1000;

			if (slot >= 10) {
				// done
				running = false;
			} else if (elapsed_ms < 2000) {
				// first 5 seconds: takeoff at 1800us
				r1 = r2 = r3 = r4 = 1500;
			} else if (in_hold) {
				switch (slot) {
					case 2: // Forward: front low, rear high
						r1 = TEST_PWM_LOW;  // FR
						r3 = TEST_PWM_LOW;  // FL
						r2 = TEST_PWM_HIGH; // RL
						r4 = TEST_PWM_HIGH; // RR
						break;
					case 4: // Right: right low, left high
						r1 = TEST_PWM_LOW;  // FR
						r4 = TEST_PWM_LOW;  // RR
						r3 = TEST_PWM_HIGH; // FL
						r2 = TEST_PWM_HIGH; // RL
						break;
					case 6: // Back: rear low, front high
						r2 = TEST_PWM_LOW;  // RL
						r4 = TEST_PWM_LOW;  // RR
						r1 = TEST_PWM_HIGH; // FR
						r3 = TEST_PWM_HIGH; // FL
						break;
					case 8: // Left: left low, right high
						r3 = TEST_PWM_LOW;  // FL
						r2 = TEST_PWM_LOW;  // RL
						r1 = TEST_PWM_HIGH; // FR
						r4 = TEST_PWM_HIGH; // RR
						break;

                        
				}
				writer.write(r1, r2, r3, r4);
			}

			// write at fixed rate
			writer.write(r1, r2, r3, r4);
			// drain latest telemetry (non-blocking)
			io.handle().consume_telem(last_seq, [&](const XSimTelemetry& d){ last_telem = d; have_telem = true; });

			// 10 Hz log
			if ((now - last_log) >= LOG_INTERVAL) {
				last_log = now;
				float roll_deg=0.0f, pitch_deg=0.0f, yaw_deg=0.0f;
				if (have_telem) {
					// compute Euler from latest quaternion
					AttitudeUtils::computeEulerDeg(
						static_cast<float>(last_telem.quat[0]),
						static_cast<float>(last_telem.quat[1]),
						static_cast<float>(last_telem.quat[2]),
						static_cast<float>(last_telem.quat[3]),
						roll_deg, pitch_deg, yaw_deg
					);
				}
				std::ostringstream os;
				os.setf(std::ios::fixed); os.precision(2);
				os << (elapsed_ms < 5000 ? "takeoff_1800" : "cardinal_tilt")
				   << " mix[FR RL FL RR]= " << r1 << " " << r2 << " " << r3 << " " << r4
				   << " att[r p y]= " << roll_deg << " " << pitch_deg << " " << yaw_deg;
				logger.logText(os.str());
			}

			next_tp += period;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	logger.logText("scenario_cardinal_tilt done");
}

// Scenario: all rotors at 1600us continuously (ascend)
static void scenario_all_1600(XSimIo& io, LogHelper& logger)
{
	RotorWriter writer(io.handle());
	auto period = std::chrono::microseconds(1000000 / PWM_TX_HZ);
	auto next_tp = std::chrono::high_resolution_clock::now() + period;
	auto last_log = std::chrono::high_resolution_clock::now();
	const auto LOG_INTERVAL = std::chrono::milliseconds(100);
    // attitude logging
    uint32_t last_seq = 0; XSimTelemetry last_telem{}; bool have_telem=false;
	for(;;) {
		auto now = std::chrono::high_resolution_clock::now();
		if (now >= next_tp) {
			writer.write(1600, 1600, 1600, 1600);
			next_tp += period;
		}
        // drain telemetry for latest attitude
        io.handle().consume_telem(last_seq, [&](const XSimTelemetry& d){ last_telem = d; have_telem = true; });
        if ((now - last_log) >= LOG_INTERVAL) {
			last_log = now;
            float roll_deg=0, pitch_deg=0, yaw_deg=0;
            if (have_telem) {
                AttitudeUtils::computeEulerDeg(
                    static_cast<float>(last_telem.quat[0]),
                    static_cast<float>(last_telem.quat[1]),
                    static_cast<float>(last_telem.quat[2]),
                    static_cast<float>(last_telem.quat[3]),
                    roll_deg, pitch_deg, yaw_deg
                );
            }
            std::ostringstream os; os.setf(std::ios::fixed); os.precision(2);
            os << "all_1600 mix[FR RL FL RR]= 1600 1600 1600 1600"
               << " att[r p y]= " << roll_deg << " " << pitch_deg << " " << yaw_deg;
            logger.logText(os.str());
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

// Scenario: 3s takeoff at 1800us, then hover with rotor4 bias +1us
static void scenario_takeoff_hover_bias(XSimIo& io, LogHelper& logger)
{
    RotorWriter writer(io.handle());
    auto period = std::chrono::microseconds(1000000 / PWM_TX_HZ);
    auto start = std::chrono::high_resolution_clock::now();
    auto next_tp = start + period;
    auto last_log = start;
    const auto LOG_INTERVAL = std::chrono::milliseconds(100);
    // attitude logging
    uint32_t last_seq = 0; XSimTelemetry last_telem{}; bool have_telem=false;

    // PID controllers for roll/pitch stabilization (small angles)
    PidController pid_roll;  pid_roll.configure(8.0f, 2.0f, 80.0f);   // roll: kp, ki, I-limit
    PidController pid_pitch; pid_pitch.configure(10.0f, 3.0f, 80.0f); // pitch: kp, ki, I-limit
    const float kd_roll_us_per_rad_s  = 2.0f;
    const float kd_pitch_us_per_rad_s = 2.0f;
    float prev_ex = 0.0f, prev_ey = 0.0f;
    auto last_pid_tp = start;

    const int TAKEOFF_US = 1800;
    const int HOVER_US   = 1600;
    const int BIAS_US    = 1;      // apply to rotor4 (RR)

    for(;;){
        auto now = std::chrono::high_resolution_clock::now();
        if (now >= next_tp) {
            int elapsed_ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
            uint16_t r1, r2, r3, r4; // FR, RL, FL, RR
            if (elapsed_ms < 3000) {
                r1 = r2 = r3 = r4 = (uint16_t)TAKEOFF_US;
                pid_roll.reset(); pid_pitch.reset();
                prev_ex = 0.0f; prev_ey = 0.0f; last_pid_tp = now;
            } else {
                // compute attitude error from telemetry (des roll=0, pitch=0, yaw=hold)
                io.handle().consume_telem(last_seq, [&](const XSimTelemetry& d){ last_telem = d; have_telem = true; });
                float roll_deg=0, pitch_deg=0, yaw_deg=0;
                if (have_telem) {
                    AttitudeUtils::computeEulerDeg(
                        static_cast<float>(last_telem.quat[0]),
                        static_cast<float>(last_telem.quat[1]),
                        static_cast<float>(last_telem.quat[2]),
                        static_cast<float>(last_telem.quat[3]),
                        roll_deg, pitch_deg, yaw_deg);
                }
                // errors in radians
                float ex = roll_deg  * 3.1415926535f / 180.0f;
                float ey = pitch_deg * 3.1415926535f / 180.0f;
                // dt for PID
                double dt_sec = std::chrono::duration<double>(now - last_pid_tp).count();
                last_pid_tp = now;
                if (dt_sec < 0.0005) dt_sec = 0.0005; else if (dt_sec > 0.01) dt_sec = 0.01;
                // PD components
                float p_roll_us  = pid_roll.compute(ex, dt_sec, true);   // PI
                float p_pitch_us = pid_pitch.compute(ey, dt_sec, true);  // PI
                float dex = static_cast<float>((ex - prev_ex) / dt_sec);
                float dey = static_cast<float>((ey - prev_ey) / dt_sec);
                prev_ex = ex; prev_ey = ey;
                float d_roll_us  = kd_roll_us_per_rad_s  * (-dex); // negative feedback
                float d_pitch_us = kd_pitch_us_per_rad_s * (-dey);
                // rounding to nearest us to avoid truncation bias
                int d_roll  = static_cast<int>(std::lround(p_roll_us  + d_roll_us));
                int d_pitch = static_cast<int>(std::lround(p_pitch_us + d_pitch_us));
                // clamp corrections
                auto clampd = [](int v, int lim){ return v > lim ? lim : (v < -lim ? -lim : v); };
                d_roll = clampd(d_roll, 20);
                d_pitch = clampd(d_pitch, 30);
                // base hover + bias + PID mixer (FR, RL, FL, RR)
                // Stabilizing signs: FR=base + d_pitch - d_roll, FL=base + d_pitch + d_roll,
                // RL=base - d_pitch + d_roll, RR=base - d_pitch - d_roll (+bias)
                r1 = static_cast<uint16_t>(HOVER_US + ( d_pitch - d_roll)); // FR
                r3 = static_cast<uint16_t>(HOVER_US + ( d_pitch + d_roll)); // FL
                r2 = static_cast<uint16_t>(HOVER_US + (-d_pitch + d_roll)); // RL
                r4 = static_cast<uint16_t>(HOVER_US + (-d_pitch - d_roll) + BIAS_US); // RR with +1us bias
                // clamp 1000..2000
                auto clamp = [](uint16_t v){ if (v < 1000) return (uint16_t)1000; if (v > 2000) return (uint16_t)2000; return v; };
                r1 = clamp(r1); r2 = clamp(r2); r3 = clamp(r3); r4 = clamp(r4);

                // 10 Hz debug PDI terms
            if ((now - last_log) >= LOG_INTERVAL) {
                    std::ostringstream os2; os2.setf(std::ios::fixed); os2.precision(2);
                    os2 << "pdi debug R[P D I]= " << p_roll_us << " " << d_roll_us << " " << pid_roll.getIntegralUs()
                        << " P= " << p_pitch_us << " D= " << d_pitch_us << " I= " << pid_pitch.getIntegralUs();
                    logger.logText(os2.str());
                }
            }
            writer.write(r1, r2, r3, r4);

            // drain telemetry for latest attitude
            io.handle().consume_telem(last_seq, [&](const XSimTelemetry& d){ last_telem = d; have_telem = true; });
            if ((now - last_log) >= LOG_INTERVAL) {
                last_log = now;
                std::ostringstream os; os.setf(std::ios::fixed); os.precision(2);
                float roll_deg=0, pitch_deg=0, yaw_deg=0;
                if (have_telem) {
                    AttitudeUtils::computeEulerDeg(
                        static_cast<float>(last_telem.quat[0]),
                        static_cast<float>(last_telem.quat[1]),
                        static_cast<float>(last_telem.quat[2]),
                        static_cast<float>(last_telem.quat[3]),
                        roll_deg, pitch_deg, yaw_deg
                    );
                }
                if (elapsed_ms < 3000) {
                    os << "takeoff_1800 mix[FR RL FL RR]= " << r1 << " " << r2 << " " << r3 << " " << r4
                       << " att[r p y]= " << roll_deg << " " << pitch_deg << " " << yaw_deg;
                } else {
                    os << "hover_bias rr+" << BIAS_US << " mix[FR RL FL RR]= " << r1 << " " << r2 << " " << r3 << " " << r4
                       << " att[r p y]= " << roll_deg << " " << pitch_deg << " " << yaw_deg;
                }
                logger.logText(os.str());
            }
            next_tp += period;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int main()
{
	std::cout << "test_client" << std::endl;
	timeBeginPeriod(1);

	LogHelper logger;
	{
		std::string path = logger.openNew("LOGS");
		if (!path.empty()) std::cout << "log file: " << path << std::endl;
	}

	XSimIo io;
	if (!io.connect("AirSimXsim")) {
		logger.logText("connect failed");
		timeEndPeriod(1);
		return 1;
	}

    // run selected scenario (extensible)
#if TEST_SCENARIO == SCENARIO_CARDINAL_TILT
    scenario_cardinal_tilt(io, logger);
#elif TEST_SCENARIO == SCENARIO_ALL_1600
    scenario_all_1600(io, logger);
#elif TEST_SCENARIO == SCENARIO_TAKEOFF_HOVER_BIAS
    scenario_takeoff_hover_bias(io, logger);
#else
#error Invalid TEST_SCENARIO selected
#endif
	timeEndPeriod(1);
	return 0;
}


