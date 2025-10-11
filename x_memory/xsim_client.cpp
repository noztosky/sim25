#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <atomic>
#include "x_xsim.h"
#include "lib/PidController.hpp"
#include "lib/LogHelper.hpp"
#include "lib/XSimIo.hpp"
#include "lib/AttitudeUtils.hpp"
#include <cstdint>
#include <random>
#include "attitude_estimator.hpp"
#include <cmath>
#include <fstream>
#include <ctime>
#include <sstream>
#include <cstdio>

static const int PWM_TX_HZ = 400;
// Debug toggles
static constexpr bool DISABLE_BIAS     = false; // ignore per-rotor bias during tuning
// Pulse test (sequential rotor bump) before PID to validate mapping
static constexpr bool ENABLE_PULSE_TEST = false;
static constexpr int  PULSE_US          = 15;    // +us bump on selected rotor (reduced)
static constexpr int  PULSE_SLOT_MS     = 200;   // duration per rotor (reduced)
// Slew rate limiting per 400Hz tick
static constexpr int  SLEW_US_PER_TICK  = 10;    // max change per tick
// Directed rotor test to validate orientation mapping (Forward, Right, Back, Left)
static constexpr bool ENABLE_TEST_ROTORS = true;
static constexpr int  TEST_PWM_LOW  = 1000;  // suggested
static constexpr int  TEST_PWM_HIGH = 1600;  // suggested
static constexpr int  TEST_HOLD_MS  = 1000;  // hold per direction
static constexpr int  TEST_GAP_MS   = 1000;  // gap between directions

static LogHelper logger;


// Test-rotors sequence helper
static bool test_rotors(
	const int bias[4],
	int& m1, int& m2, int& m3, int& m4,
	const std::chrono::high_resolution_clock::time_point& now,
	std::chrono::high_resolution_clock::time_point& test_start,
	bool& started,
	bool& done)
{
    if (!ENABLE_TEST_ROTORS) return false;
    if (done) return false;
    if (!started) { started = true; test_start = now; logger.logText("test_rotors start"); }
	int elapsed_ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - test_start).count();
	int slot_len = TEST_HOLD_MS + TEST_GAP_MS; // 2000ms
	int slot = elapsed_ms / slot_len; // 0..7
	int in_slot_ms = elapsed_ms % slot_len;
    if (slot >= 8) { done = true; logger.logText("test_rotors done"); return false; }
	if (in_slot_ms >= TEST_HOLD_MS) return false; // gap period
	// slots: 0:F,1:gap,2:R,3:gap,4:B,5:gap,6:L,7:gap
	if (slot == 0) {
		m1 = TEST_PWM_LOW  + bias[0]; // FR low
		m3 = TEST_PWM_LOW  + bias[2]; // FL low
		m2 = TEST_PWM_HIGH + bias[1]; // RL high
		m4 = TEST_PWM_HIGH + bias[3]; // RR high
		return true;
	}
	if (slot == 2) {
		m1 = TEST_PWM_LOW  + bias[0]; // FR low
		m4 = TEST_PWM_LOW  + bias[3]; // RR low
		m3 = TEST_PWM_HIGH + bias[2]; // FL high
		m2 = TEST_PWM_HIGH + bias[1]; // RL high
		return true;
	}
	if (slot == 4) {
		m2 = TEST_PWM_LOW  + bias[1]; // RL low
		m4 = TEST_PWM_LOW  + bias[3]; // RR low
		m1 = TEST_PWM_HIGH + bias[0]; // FR high
		m3 = TEST_PWM_HIGH + bias[2]; // FL high
		return true;
	}
	if (slot == 6) {
		m3 = TEST_PWM_LOW  + bias[2]; // FL low
		m2 = TEST_PWM_LOW  + bias[1]; // RL low
		m1 = TEST_PWM_HIGH + bias[0]; // FR high
		m4 = TEST_PWM_HIGH + bias[3]; // RR high
		return true;
	}
	return false;
}
 

// Helper: write rotor PWMs to shared memory
static void writeRotors(x_xsim& xs, uint16_t r1, uint16_t r2, uint16_t r3, uint16_t r4, int& seq, XSimPwm* last_out = nullptr)
{
    auto now = std::chrono::high_resolution_clock::now();
    XSimPwm p{};
    p.rotor1 = static_cast<int>(r1);
    p.rotor2 = static_cast<int>(r2);
    p.rotor3 = static_cast<int>(r3);
    p.rotor4 = static_cast<int>(r4);
    p.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    p.seq = ++seq;
    p.is_valid = true;
    std::memset(p.padding, 0, sizeof p.padding);
    xs.submit_pwm(p);
    if (last_out) *last_out = p;
}

// Helper: poll telemetry from shared memory and update counters
static void pollTelemetry(
    x_xsim& xs,
    uint32_t& last_seq,
    XSimTelemetry& latest,
    bool& have_telem,
    std::atomic<uint64_t>& imu_rx_cnt,
    std::atomic<uint64_t>& mag_rx_cnt,
    std::atomic<uint64_t>& baro_rx_cnt,
    std::atomic<uint64_t>& loc_rx_cnt,
    AttitudeEstimator& estimator,
    long long& last_ts_ns)
{
    static int seq_local = 0;
    xs.consume_telem(last_seq, [&](const XSimTelemetry& d){
        latest = d;
        have_telem = true;
        imu_rx_cnt.fetch_add(1, std::memory_order_relaxed);
        if(((++seq_local) % 10) == 0){
            mag_rx_cnt.fetch_add(1, std::memory_order_relaxed);
            baro_rx_cnt.fetch_add(1, std::memory_order_relaxed);
            loc_rx_cnt.fetch_add(1, std::memory_order_relaxed);
        }
        // attitude estimation at telemetry rate (~1kHz)
        float gyro[3] = { static_cast<float>(d.gyro[0]), static_cast<float>(d.gyro[1]), static_cast<float>(d.gyro[2]) };
        float acc[3]  = { static_cast<float>(d.acc[0]),  static_cast<float>(d.acc[1]),  static_cast<float>(d.acc[2]) };
        float dt = 0.001f;
        if (last_ts_ns != 0) {
            long long dtns = d.timestamp - last_ts_ns;
            if (dtns > 0 && dtns < 10000000LL) dt = static_cast<float>(dtns * 1e-9);
        }
        last_ts_ns = d.timestamp;
        // clamp dt to [0.5ms, 2.5ms] for stability
        if (dt < 0.0005f) dt = 0.0005f; else if (dt > 0.0025f) dt = 0.0025f;
        estimator.update(gyro, acc, dt);
    });
}

// Helper: print one-line stats and reset counters
 

int main(){
    std::cout << "xsim_client" << std::endl;
    timeBeginPeriod(1);

    // create LOGS/timestamp.LOG via helper
    {
        std::string path = logger.openNew("LOGS");
        if (!path.empty()) std::cout << "log file: " << path << std::endl;
    }

    XSimIo io;
    if(!io.connect("AirSimXsim")){
        std::cerr << "xsim_client: connect failed" << std::endl;
        timeEndPeriod(1);
        return 1;
    }

    // consume telemetry & attitude estimation (1 kHz)
    std::atomic<bool> running{true};
    // last received values and counters for Hz
    XSimTelemetry last_telem{}; bool have_telem=false;
    std::atomic<uint64_t> imu_rx_cnt{0}, mag_rx_cnt{0}, baro_rx_cnt{0}, loc_rx_cnt{0};
    // shared estimated quaternion (from rx thread)
    struct { std::atomic<float> w{1.0f}, x{0.0f}, y{0.0f}, z{0.0f}; } q_est;
    std::thread rx_thread([&]{
        uint32_t last_seq = 0;
        AttitudeEstimator estimator; long long last_ts_ns = 0;
        while(running.load()){
            pollTelemetry(io.handle(), last_seq, last_telem, have_telem, imu_rx_cnt, mag_rx_cnt, baro_rx_cnt, loc_rx_cnt, estimator, last_ts_ns);
            // publish latest quaternion to shared
            float w,x,y,z; estimator.getQuaternion(w,x,y,z);
            q_est.w.store(w, std::memory_order_relaxed);
            q_est.x.store(x, std::memory_order_relaxed);
            q_est.y.store(y, std::memory_order_relaxed);
            q_est.z.store(z, std::memory_order_relaxed);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    // transmit pwm at 400 Hz
    auto period = std::chrono::microseconds(1000000/PWM_TX_HZ);
    auto start_time = std::chrono::high_resolution_clock::now();
    auto liftoff_time = start_time + std::chrono::seconds(3);
    auto pid_enable_time = liftoff_time; // no extra PID delay after liftoff
    auto next_tp = start_time + period;
	int seq=0; int base=1000; bool armed=false;
    // fixed per-rotor bias [1..3] us to emulate motor variance
    std::random_device rd; std::mt19937 gen(rd()); std::uniform_int_distribution<int> dist(1,3);
	const int bias[4] = { DISABLE_BIAS ? 0 : 1, DISABLE_BIAS ? 0 : 0, DISABLE_BIAS ? 0 : 0, DISABLE_BIAS ? 0 : 0 };

    auto last_print = std::chrono::high_resolution_clock::now();
	auto last_ctrl_log = last_print;
	XSimPwm last{}; bool have=false;
	// pulse test state
	bool pulse_started=false; bool pulse_done=false; int pulse_idx=0;
	auto pulse_slot_start = start_time;
	// rotor direction test state
	bool test_started=false; bool test_done=false;
	auto test_start = start_time;

    while(true){
        auto now = std::chrono::high_resolution_clock::now();
        if(now >= next_tp){
            if(!armed && now >= liftoff_time){ base=1600; armed=true; }
            auto clamp = [](int v){ return v < 1000 ? 1000 : (v > 2000 ? 2000 : v); };

			int m1, m2, m3, m4;
			// pulse test scheduling (runs only once, right after liftoff)
			if (ENABLE_PULSE_TEST && armed && !pulse_done) {
                if (!pulse_started) { pulse_started = true; pulse_idx = 0; pulse_slot_start = now; 
                    logger.logText("pulse_test start");
				}
				else if (std::chrono::duration_cast<std::chrono::milliseconds>(now - pulse_slot_start).count() >= PULSE_SLOT_MS) {
					pulse_idx++; pulse_slot_start = now;
                    if (pulse_idx >= 4) { pulse_done = true; logger.logText("pulse_test done"); }
				}
			}

			// optional rotor test (Forward, Right, Back, Left) overrides PID during hold window
			bool did_test = false;
			if (ENABLE_TEST_ROTORS && armed && !test_done) {
				int tm1=0, tm2=0, tm3=0, tm4=0;
				if (test_rotors(bias, tm1, tm2, tm3, tm4, now, test_start, test_started, test_done)) {
					m1 = clamp(tm1); m2 = clamp(tm2); m3 = clamp(tm3); m4 = clamp(tm4);
					did_test = true;
					// light log (reuse control log cadence)
					if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_ctrl_log).count() >= 500) {
						last_ctrl_log = now;
						logger.logText("test_rotors mix[FR RL FL RR]= " + std::to_string(m1) + " " + std::to_string(m2) + " " + std::to_string(m3) + " " + std::to_string(m4));
					}
				}
			}

			bool pid_phase = armed && (now >= pid_enable_time) && !did_test && (!ENABLE_PULSE_TEST || pulse_done);
			if (pid_phase) {
                // read estimated quaternion
                float qw = q_est.w.load(std::memory_order_relaxed);
                float qx = q_est.x.load(std::memory_order_relaxed);
                float qy = q_est.y.load(std::memory_order_relaxed);
                float qz = q_est.z.load(std::memory_order_relaxed);
                // compute yaw from q_est and desired yaw quaternion
                float yaw = AttitudeUtils::computeYawRad(qw, qx, qy, qz);
                float qd_w, qd_x, qd_y, qd_z; AttitudeUtils::makeYawQuat(yaw, qd_w, qd_x, qd_y, qd_z);
                // small-angle error ex,ey via helper (q_des * inverse(q_est))
                float ex, ey; AttitudeUtils::computeSmallAngleErrorExEy(qd_w, qd_x, qd_y, qd_z, qw, qx, qy, qz, ex, ey);
                // simple PI (pitch-only I) to PWM us with fade-in after enable
                const float kp_us_per_rad = 20.0f; // modest P gain
                const float ki_us_per_rad_s = 2.0f; // small I (pitch only)
                // negative feedback
                float d_roll_f = kp_us_per_rad * (-ex);
                float d_pitch_f = kp_us_per_rad * (-ey);
                // integrator on roll/pitch error to cancel slow bias
                static double i_roll_us = 0.0, i_pitch_us = 0.0;
                static auto last_pid_tp = std::chrono::high_resolution_clock::time_point{};
                double dt_pid = 1.0/400.0;
                if (last_pid_tp.time_since_epoch().count() != 0) {
                    dt_pid = std::chrono::duration<double>(now - last_pid_tp).count();
                    if (dt_pid < 0.0005) dt_pid = 0.0005; else if (dt_pid > 0.01) dt_pid = 0.01;
                }
                last_pid_tp = now;
				// anti-windup: pause integral if P-term near clamp
				bool sat_roll_p = (std::abs(d_roll_f)  >= 70.0f);
				bool sat_pitch_p = (std::abs(d_pitch_f) >= 70.0f);
				if (!sat_roll_p)  i_roll_us  += ki_us_per_rad_s  * (-ex) * dt_pid; else i_roll_us  *= 0.98; // slight decay when saturated
				if (!sat_pitch_p) i_pitch_us += ki_us_per_rad_s  * (-ey) * dt_pid; else i_pitch_us *= 0.98;
                // clamp I term
                auto clampr = [](double v, double lim){ return v > lim ? lim : (v < -lim ? -lim : v); };
				i_roll_us  = clampr(i_roll_us,  40.0);
				i_pitch_us = clampr(i_pitch_us, 40.0);
                // fade-in over 1.0s from pid_enable_time
                float fade = 1.0f;
                if (now < pid_enable_time + std::chrono::seconds(2)) { // longer fade-in
                    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - pid_enable_time).count();
                    if (ms < 0) ms = 0; if (ms > 2000) ms = 2000;
                    fade = static_cast<float>(ms) / 2000.0f;
                }
                int d_roll = static_cast<int>((d_roll_f + i_roll_us) * fade);
                int d_pitch = static_cast<int>((d_pitch_f + i_pitch_us) * fade);
                // clamp incremental correction (roll tighter than pitch)
                auto clampd = [](int v, int lim){ return v > lim ? lim : (v < -lim ? -lim : v); };
                d_roll = clampd(d_roll, 20);
                d_pitch = clampd(d_pitch, 30);
                // Canonical QuadX mixer (FR, RL, FL, RR)
                // roll right (+) => decrease right, increase left; pitch up (+) => decrease front, increase rear
                // Using d_* already negative feedback of errors
                m1 = clamp(base + bias[0] + (-d_pitch - d_roll)); // front-right
                m2 = clamp(base + bias[1] + ( d_pitch - d_roll)); // rear-left
                m3 = clamp(base + bias[2] + (-d_pitch + d_roll)); // front-left
                m4 = clamp(base + bias[3] + ( d_pitch + d_roll)); // rear-right

				// 10Hz debug control log via helper
				if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_ctrl_log).count() >= 100) {
					last_ctrl_log = now;
					// derive Euler (deg) via helper
					float roll_deg, pitch_deg, yaw_deg;
					AttitudeUtils::computeEulerDeg(qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg);
					logger.logControl(
						roll_deg, pitch_deg, yaw_deg,
						ex, ey, d_roll, d_pitch,
						i_roll_us, i_pitch_us,
						m1, m2, m3, m4,
						base, bias, kp_us_per_rad);
				}
            } else {
				// pre-PID phase: base + bias (+ optional pulse-test bump on one rotor)
				int pulse_boost[4] = {0,0,0,0};
				if (ENABLE_PULSE_TEST && pulse_started && !pulse_done && pulse_idx >= 0 && pulse_idx < 4) {
					pulse_boost[pulse_idx] = PULSE_US;
				}
				m1 = clamp(base + bias[0] + pulse_boost[0]);
				m2 = clamp(base + bias[1] + pulse_boost[1]);
				m3 = clamp(base + bias[2] + pulse_boost[2]);
				m4 = clamp(base + bias[3] + pulse_boost[3]);

                // 10Hz pulse-test log via helper
				if (ENABLE_PULSE_TEST && std::chrono::duration_cast<std::chrono::milliseconds>(now - last_ctrl_log).count() >= 100) {
					last_ctrl_log = now;
                    logger.logText(
                        std::string("pulse: idx= ") + std::to_string(pulse_started && !pulse_done ? (pulse_idx+1) : 0)
                        + " +us= " + std::to_string(pulse_started && !pulse_done ? PULSE_US : 0)
                        + ", mix[FR RL FL RR]= " + std::to_string(m1) + " " + std::to_string(m2) + " " + std::to_string(m3) + " " + std::to_string(m4)
                    );
				}
            }
            // slew-rate limit vs last output to avoid sudden steps
            auto slew = [&](int prev, int target){
                int p = prev;
                int d = target - p;
                if (d > SLEW_US_PER_TICK) d = SLEW_US_PER_TICK; else if (d < -SLEW_US_PER_TICK) d = -SLEW_US_PER_TICK;
                return p + d;
            };
            int prev1 = have ? last.rotor1 : (base + bias[0]);
            int prev2 = have ? last.rotor2 : (base + bias[1]);
            int prev3 = have ? last.rotor3 : (base + bias[2]);
            int prev4 = have ? last.rotor4 : (base + bias[3]);
            m1 = slew(prev1, m1);
            m2 = slew(prev2, m2);
            m3 = slew(prev3, m3);
            m4 = slew(prev4, m4);
            const uint16_t r1 = static_cast<uint16_t>(m1);
            const uint16_t r2 = static_cast<uint16_t>(m2);
            const uint16_t r3 = static_cast<uint16_t>(m3);
            const uint16_t r4 = static_cast<uint16_t>(m4);
            writeRotors(io.handle(), r1, r2, r3, r4, seq, &last);
            have=true;
            next_tp += period;
        }

        if(std::chrono::duration_cast<std::chrono::seconds>(now - last_print).count() >= 1){
            double dt = std::chrono::duration<double>(now - last_print).count();
            last_print = now;
            logger.logStats(last_telem, have_telem, last, have, dt, imu_rx_cnt, mag_rx_cnt, baro_rx_cnt, loc_rx_cnt, io.handle());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    running.store(false);
    if(rx_thread.joinable()) rx_thread.join();
    timeEndPeriod(1);
    return 0;
}


