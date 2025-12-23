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
#include <string>

#include "x_xsim.h"
#include "lib/LogHelper.hpp"
#include "lib/XSimIo.hpp"
#include "lib/RotorWriter.hpp"
#include "lib/AttitudeUtils.hpp"
#include "lib/PidController.hpp"
#include "../attitude_estimator.hpp"

// 공용 PWM/시간 상수
static const int TAKEOFF_US = 1800;
static const int HOVER_US   = 1600;
static const int TAKEOFF_MS = 7000; // milder takeoff duration

// Scenario selection
#define SCENARIO_CARDINAL_TILT     1
#define SCENARIO_ALL_1600          2
#define SCENARIO_TAKEOFF_HOVER_BIAS 3

#ifndef TEST_SCENARIO
#define TEST_SCENARIO           SCENARIO_TAKEOFF_HOVER_BIAS
#endif

static const int PWM_TX_HZ = 400;

// Summary log prefix (scenario tag shown in 10 Hz lines)
static std::string g_summary_prefix = "hover_bias";

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
				// 첫 구간: 명시된 TAKEOFF_US로 이륙 (예: 1800us)
				r1 = r2 = r3 = r4 = TAKEOFF_US;
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
    // Mahony attitude estimator state (updated at ~1 kHz)
    AttitudeEstimator estimator;
    long long last_ts_ns = 0;
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
static void scenario_takeoff_hover_bias(XSimIo& io, LogHelper& logger, int max_duration_ms = -1)
{
    RotorWriter writer(io.handle());
    auto period = std::chrono::microseconds(1000000 / PWM_TX_HZ);
    auto start = std::chrono::high_resolution_clock::now();
    auto next_tp = start + period;
    // split logging: debug 50Hz, summary 10Hz
    auto last_debug = start;
    auto last_summary = start;
    const auto DEBUG_INTERVAL = std::chrono::milliseconds(20);
    const auto SUMMARY_INTERVAL = std::chrono::milliseconds(100);
    // attitude logging
    uint32_t last_seq = 0; XSimTelemetry last_telem{}; bool have_telem=false;

    // Two-loop control: Attitude P (outer) -> Rate PID (inner)
    // Outer loop (angle P): rate_target = ANG_P * angle_error (rad/s)
    const float ANG_P_R = 4.5f;  // roll angle P [s^-1]
    const float ANG_P_P = 5.0f;  // pitch angle P [s^-1]
    // Inner loop (rate PID): units in us per (rad/s)
    float roll_kp = 50.0f, roll_ki = 12.0f,  roll_i_lim = 180.0f;
    float pitch_kp = 60.0f, pitch_ki = 14.0f, pitch_i_lim = 220.0f;
    PidController pid_roll;  pid_roll.configure(roll_kp,  roll_ki,  roll_i_lim);
    PidController pid_pitch; pid_pitch.configure(pitch_kp, pitch_ki, pitch_i_lim);
    // Gyro-based D gains (yaw uses separate small PD below)
    float kd_roll_us_per_rad_s  = 6.0f;
    float kd_pitch_us_per_rad_s = 7.0f;
    auto last_pid_tp = start;
    // gyro LPF state for D-term (PT2)
    static float gyro_x_filt = 0.0f, gyro_y_filt = 0.0f;
    static float gyro_x_filt2 = 0.0f, gyro_y_filt2 = 0.0f;
    const float GYRO_LPF_ALPHA = 0.5f;  // stage1
    const float GYRO_LPF_ALPHA2 = 0.5f; // stage2
    // fixed trims (captured once after takeoff)
    static float trim_roll_us = 0.0f, trim_pitch_us = 0.0f;
    static float trim_target_roll_us = 0.0f, trim_target_pitch_us = 0.0f; // ramp targets
    const float TRIM_LIMIT_US = 50.0f;
    const int TRIM_WINDOW_MS = 3000; // collect for 3s after takeoff
    bool trim_fixed = false;
    bool trim_ramp_active = false;
    auto trim_ramp_start = start;
    const int TRIM_RAMP_MS = 800; // ramp in over 0.8s to avoid step
    double trim_ex_sum = 0.0, trim_ey_sum = 0.0; int trim_count = 0;
    const float TRIM_K_US_PER_RAD = 80.0f; // convert avg angle error -> microseconds

    // Per-rotor biases (FR, RL, FL, RR)
    const int BIAS_FR = 1;
    const int BIAS_RL = 3;
    const int BIAS_FL = 5;
    const int BIAS_RR = 6;

    long long last_ts_ns = 0;
    AttitudeEstimator estimator;
    // diagnostic/estimation control
    const bool DIAG_MODE = false; // default 10 Hz summary only; set true for 50 Hz diag
    float est_kp_cur = 0.5f, est_ki_cur = 0.005f;
    float est_kp_cmd = est_kp_cur, est_ki_cmd = est_ki_cur; // smoothed towards this
    estimator.setGains(est_kp_cur, est_ki_cur); // softer accel trust by default
    float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
    float last_acc_norm = 9.81f;
    // latest gyro (bias-corrected) for D-term
    float latest_gyro_x = 0.0f, latest_gyro_y = 0.0f, latest_gyro_z = 0.0f;

    // Auto-tuner (simple bias/oscillation adaptive tweaks)
    const bool ENABLE_AUTO_TUNE = false; // disable by default to avoid divergence
    auto last_autotune_tp = start;
    double ex_sum = 0.0, ey_sum = 0.0; int e_count = 0;
    int roll_sign_flips = 0, pitch_sign_flips = 0; int last_roll_sign = 0, last_pitch_sign = 0;
    const float EX_BIAS_TH = 0.02f;  // ~1.15 deg
    const float EY_BIAS_TH = 0.02f;  // ~1.15 deg
    const int   FLIP_TH = 10;        // many sign flips in 1s => oscillatory
    const float KP_STEP = 0.5f, KI_STEP_R = 0.3f, KI_STEP_P = 0.5f, KD_STEP = 0.2f, ILIM_STEP = 10.0f;
    const float R_KI_MAX = 8.0f, P_KI_MAX = 10.0f, R_ILIM_MAX = 200.0f, P_ILIM_MAX = 220.0f, KD_MAX = 3.5f, KP_MIN = 6.0f;
    // yaw hold reference (set on first hover tick)
    bool yaw_ref_set = false;
    float yaw_ref_rad = 0.0f;
    // Yaw: angle + rate hold (small Kp to limit reliance on yaw angle)
    const float kp_yaw_us_per_rad = 0.0f; // rate-only yaw hold (ignore angle)
    const float kd_yaw_us_per_rad_s = 250.0f; // stronger rate damping
    const float ki_yaw_us_per_rad_s = 180.0f; // integral to remove steady bias
    const float yaw_i_limit_us = 120.0f;      // anti-windup clamp for I term
    const float YAW_RATE_SIGN =  1.0f; // polarity (flip to opposite if needed)
    static float last_cy_us = 0.0f; // for 10 Hz summary logging
    static float yaw_i_accum_us = 0.0f; // yaw PI integrator (in microseconds)
    static float last_cp_us = 0.0f, last_cr_us = 0.0f; // for 10 Hz summary logging
    // Smooth EST/GT blend for control errors (avoid step source switching)
    float gt_blend = 0.0f; // 0: EST only, 1: GT only


    for(;;){
        auto now = std::chrono::high_resolution_clock::now();
        if (now >= next_tp) {
            int elapsed_ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
            uint16_t r1, r2, r3, r4; // FR, RL, FL, RR
            if (elapsed_ms < TAKEOFF_MS) {
                r1 = r2 = r3 = r4 = (uint16_t)TAKEOFF_US;
                pid_roll.reset(); pid_pitch.reset();
                yaw_i_accum_us = 0.0f; // reset yaw integral during takeoff
                last_pid_tp = now;
            } else {
                // dt for control updates (used by smoothing and PIDs)
                double dt_sec = std::chrono::duration<double>(now - last_pid_tp).count();
                last_pid_tp = now;
                if (dt_sec < 0.0005) dt_sec = 0.0005; else if (dt_sec > 0.01) dt_sec = 0.01;
                // Adaptive Mahony gains: first 1s after takeoff strong accel trust,
                // then gate by accel magnitude to avoid dynamic under/over-correction
                if (elapsed_ms < (TAKEOFF_MS + 1000)) {
                    est_kp_cmd = 2.0f; est_ki_cmd = 0.02f;
                } else {
                    const float g = 9.80665f;
                    float aerr = std::fabs(last_acc_norm - g);
                    if (aerr < 0.3f) {            // very near 1g -> trust accel more
                        est_kp_cmd = 0.8f; est_ki_cmd = 0.008f;
                    } else if (aerr < 0.8f) {     // moderate dynamics
                        est_kp_cmd = 0.5f; est_ki_cmd = 0.005f;
                    } else {                       // strong dynamics -> rely more on gyro
                        est_kp_cmd = 0.2f; est_ki_cmd = 0.002f;
                    }
                }
                // Smooth gains towards commands to avoid steps
                {
                    float tau = 0.35f; // seconds
                    float alpha = static_cast<float>(dt_sec) / tau;
                    if (alpha < 0.0f) alpha = 0.0f; else if (alpha > 0.25f) alpha = 0.25f; // cap per-tick blend
                    est_kp_cur += alpha * (est_kp_cmd - est_kp_cur);
                    est_ki_cur += alpha * (est_ki_cmd - est_ki_cur);
                }
                estimator.setGains(est_kp_cur, est_ki_cur);
                // Drain telemetry and update Mahony estimator for each new sample (~1 kHz)
                io.handle().consume_telem(last_seq, [&](const XSimTelemetry& d){
                    last_telem = d; have_telem = true;
                    float gyro[3] = { static_cast<float>(d.gyro[0]), static_cast<float>(d.gyro[1]), static_cast<float>(d.gyro[2]) };
                    float acc[3]  = { static_cast<float>(d.acc[0]),  static_cast<float>(d.acc[1]),  static_cast<float>(d.acc[2]) };
                    float dt = 0.001f;
                    if (last_ts_ns != 0) {
                        long long dtns = d.timestamp - last_ts_ns;
                        if (dtns > 0 && dtns < 10000000LL) dt = static_cast<float>(dtns * 1e-9f);
                    }
                    last_ts_ns = d.timestamp;
                    if (dt < 0.0005f) dt = 0.0005f; else if (dt > 0.0025f) dt = 0.0025f;
                    // online gyro bias estimation when near static (low gyro, accel ~1g)
                    float an = std::sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
                    last_acc_norm = an;
                    float gyro_abs = std::fabs(gyro[0]) + std::fabs(gyro[1]) + std::fabs(gyro[2]);
                    if (gyro_abs < 0.15f && std::fabs(an - 9.80665f) < 1.0f) {
                        const float beta = 0.001f; // slow EMA
                        gyro_bias[0] = (1.0f - beta) * gyro_bias[0] + beta * gyro[0];
                        gyro_bias[1] = (1.0f - beta) * gyro_bias[1] + beta * gyro[1];
                        gyro_bias[2] = (1.0f - beta) * gyro_bias[2] + beta * gyro[2];
                    }
                    float gyro_corr[3] = { gyro[0] - gyro_bias[0], gyro[1] - gyro_bias[1], gyro[2] - gyro_bias[2] };
                    // store for D-term use
                    latest_gyro_x = gyro_corr[0]; latest_gyro_y = gyro_corr[1]; latest_gyro_z = gyro_corr[2];
                    estimator.update(gyro_corr, acc, dt);
                });
                // Use estimator attitude for control (yaw-hold via quaternion error)
                float qw=1, qx=0, qy=0, qz=0; estimator.getQuaternion(qw,qx,qy,qz);
                float yaw_rad = AttitudeUtils::computeYawRad(qw,qx,qy,qz);
                float qd_w=1, qd_x=0, qd_y=0, qd_z=0;
                AttitudeUtils::makeYawQuat(yaw_rad, qd_w, qd_x, qd_y, qd_z);
                // estimator-based small-angle errors
                float ex_est = 0.0f, ey_est = 0.0f;
                AttitudeUtils::computeSmallAngleErrorExEy(qd_w, qd_x, qd_y, qd_z, qw, qx, qy, qz, ex_est, ey_est);
                // ground-truth-based small-angle errors (for diagnostics/control toggle)
                float ex_gt = 0.0f, ey_gt = 0.0f;
                if (have_telem) {
                    float gtw = static_cast<float>(last_telem.quat[0]);
                    float gtx = static_cast<float>(last_telem.quat[1]);
                    float gty = static_cast<float>(last_telem.quat[2]);
                    float gtz = static_cast<float>(last_telem.quat[3]);
                    AttitudeUtils::computeSmallAngleErrorExEy(qd_w, qd_x, qd_y, qd_z, gtw, gtx, gty, gtz, ex_gt, ey_gt);
                }
                // Compute GT Euler and quaternion error for safety gating
                float gt_r_deg = 0.0f, gt_p_deg = 0.0f, gt_y_deg = 0.0f;
                float qerr_deg_ctrl = 0.0f;
                if (have_telem) {
                    AttitudeUtils::computeEulerDeg(
                        static_cast<float>(last_telem.quat[0]),
                        static_cast<float>(last_telem.quat[1]),
                        static_cast<float>(last_telem.quat[2]),
                        static_cast<float>(last_telem.quat[3]),
                        gt_r_deg, gt_p_deg, gt_y_deg);
                    float qgtw = static_cast<float>(last_telem.quat[0]);
                    float qgtx = static_cast<float>(last_telem.quat[1]);
                    float qgty = static_cast<float>(last_telem.quat[2]);
                    float qgtz = static_cast<float>(last_telem.quat[3]);
                    float dot = std::fabs(qw*qgtw + qx*qgtx + qy*qgty + qz*qgtz);
                    if (dot > 1.0f) dot = 1.0f; else if (dot < -1.0f) dot = -1.0f;
                    qerr_deg_ctrl = 2.0f * std::acos(dot) * 57.2957795f;
                }
                // Smoothly blend EST/GT sources instead of step switching
                float gt_w_cmd = (have_telem && (qerr_deg_ctrl > 5.0f || std::fabs(gt_r_deg) > 5.0f || std::fabs(gt_p_deg) > 5.0f)) ? 1.0f : 0.0f;
                {
                    float tau_blend = 0.25f; // seconds
                    float alpha_b = static_cast<float>(dt_sec) / tau_blend;
                    if (alpha_b < 0.0f) alpha_b = 0.0f; else if (alpha_b > 0.25f) alpha_b = 0.25f;
                    gt_blend += alpha_b * (gt_w_cmd - gt_blend);
                }
                float one_m = 1.0f - gt_blend;
                float ex = one_m * ex_est + gt_blend * ex_gt;
                float ey = one_m * ey_est + gt_blend * ey_gt;
                if (!yaw_ref_set) { yaw_ref_set = true; yaw_ref_rad = yaw_rad; }
                // PI components (with deadzone and conditional integral)
                auto applyDeadzone = [](float v){ return (std::fabs(v) < 0.005f) ? 0.0f : v; };
                float ex_eff = applyDeadzone(ex);
                float ey_eff = applyDeadzone(ey);
                bool allow_i_r = (std::fabs(ex_eff) < 0.35f);
                bool allow_i_p = (std::fabs(ey_eff) < 0.35f);
                // Outer loop: angle -> rate targets
                float wx_ref = ANG_P_R * ex_eff; // rad/s
                float wy_ref = ANG_P_P * ey_eff; // rad/s
                // reset integrators only on very large tilt to avoid premature windup clearing
                if (std::fabs(ex_eff) > 0.60f || std::fabs(ey_eff) > 0.60f) { pid_roll.reset(); pid_pitch.reset(); }
                // Inner loop PI on rate error
                float erx = wx_ref - latest_gyro_x;
                float ery = wy_ref - latest_gyro_y;
                float p_roll_us  = pid_roll.compute(erx, dt_sec, allow_i_r);   // rate PI
                float p_pitch_us = pid_pitch.compute(ery, dt_sec, allow_i_p);  // rate PI
                // D components from gyro (bias-corrected) with PT2 LPF and clamp
                gyro_x_filt = GYRO_LPF_ALPHA  * gyro_x_filt  + (1.0f - GYRO_LPF_ALPHA)  * latest_gyro_x;
                gyro_y_filt = GYRO_LPF_ALPHA  * gyro_y_filt  + (1.0f - GYRO_LPF_ALPHA)  * latest_gyro_y;
                gyro_x_filt2 = GYRO_LPF_ALPHA2 * gyro_x_filt2 + (1.0f - GYRO_LPF_ALPHA2) * gyro_x_filt;
                gyro_y_filt2 = GYRO_LPF_ALPHA2 * gyro_y_filt2 + (1.0f - GYRO_LPF_ALPHA2) * gyro_y_filt;
                auto clampd = [](float v, float lim){ if (v > lim) return lim; if (v < -lim) return -lim; return v; };
                float d_roll_us  = clampd(kd_roll_us_per_rad_s  * (-gyro_x_filt2), 12.0f); // tighter clamp
                float d_pitch_us = clampd(kd_pitch_us_per_rad_s * (-gyro_y_filt2), 12.0f);
                // combined corrections (float throughout)
                float c_roll_us  = p_roll_us  + d_roll_us;
                float c_pitch_us = p_pitch_us + d_pitch_us;
                // trim capture window: collect 3s after takeoff, then fix trims (with ramp-in)
                if (!trim_fixed) {
                    if (elapsed_ms >= TAKEOFF_MS && elapsed_ms < (TAKEOFF_MS + TRIM_WINDOW_MS)) {
                        trim_ex_sum += static_cast<double>(ex_eff);
                        trim_ey_sum += static_cast<double>(ey_eff);
                        trim_count++;
                    } else if (elapsed_ms >= (TAKEOFF_MS + TRIM_WINDOW_MS)) {
                        float ex_avg = (trim_count > 0) ? static_cast<float>(trim_ex_sum / trim_count) : 0.0f;
                        float ey_avg = (trim_count > 0) ? static_cast<float>(trim_ey_sum / trim_count) : 0.0f;
                        trim_target_roll_us  = std::max(-TRIM_LIMIT_US, std::min(TRIM_LIMIT_US, -TRIM_K_US_PER_RAD * ex_avg));
                        trim_target_pitch_us = std::max(-TRIM_LIMIT_US, std::min(TRIM_LIMIT_US, -TRIM_K_US_PER_RAD * ey_avg));
                        trim_ramp_active = true; trim_ramp_start = now; trim_fixed = true;
                    }
                }
                // apply trim ramp-in
                if (trim_ramp_active) {
                    auto ramp_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - trim_ramp_start).count();
                    float a = ramp_elapsed <= 0 ? 0.0f : (ramp_elapsed >= TRIM_RAMP_MS ? 1.0f : static_cast<float>(ramp_elapsed) / static_cast<float>(TRIM_RAMP_MS));
                    trim_roll_us  = a * trim_target_roll_us;
                    trim_pitch_us = a * trim_target_pitch_us;
                    if (a >= 1.0f) trim_ramp_active = false;
                }

                // feed ex/ey to auto-tuner statistics
                if (ENABLE_AUTO_TUNE && elapsed_ms >= TAKEOFF_MS) {
                    ex_sum += static_cast<double>(ex);
                    ey_sum += static_cast<double>(ey);
                    e_count++;
                    int rs = (ex > 0.0f) ? 1 : (ex < 0.0f ? -1 : 0);
                    int ps = (ey > 0.0f) ? 1 : (ey < 0.0f ? -1 : 0);
                    if (last_roll_sign != 0 && rs != 0 && rs != last_roll_sign) roll_sign_flips++;
                    if (last_pitch_sign != 0 && ps != 0 && ps != last_pitch_sign) pitch_sign_flips++;
                    if (rs != 0) last_roll_sign = rs;
                    if (ps != 0) last_pitch_sign = ps;
                }
                // Apply signs to match mixer convention
                const float PI_SIGN_R = -1.0f; // roll
                const float PI_SIGN_P = -1.0f; // pitch
                const float Y_SIGN    =  1.0f; // enable yaw (rate hold)
                float cr = (PI_SIGN_R * c_roll_us)  + trim_roll_us;
                float cp = (PI_SIGN_P * c_pitch_us) + trim_pitch_us;
                // clamp corrections (floats) on actual commands
                auto clampdf = [](float v, float lim){ return v > lim ? lim : (v < -lim ? -lim : v); };
                cr = clampdf(cr, 300.0f);
                cp = clampdf(cp, 300.0f);
                last_cr_us = cr; last_cp_us = cp;
                // base hover + bias + PID mixer (FR, RL, FL, RR)
                // Stabilizing signs: FR=base + d_pitch - d_roll, FL=base + d_pitch + d_roll,
                // RL=base - d_pitch + d_roll, RR=base - d_pitch - d_roll (+bias)
                float r1f = static_cast<float>(HOVER_US) + ( cp - cr) + static_cast<float>(BIAS_FR); // FR
                float r3f = static_cast<float>(HOVER_US) + ( cp + cr) + static_cast<float>(BIAS_FL); // FL
                float r2f = static_cast<float>(HOVER_US) + (-cp + cr) + static_cast<float>(BIAS_RL); // RL
                float r4f = static_cast<float>(HOVER_US) + (-cp - cr) + static_cast<float>(BIAS_RR); // RR
                // yaw PD (hold yaw to yaw_ref)
                auto wrap_pi = [](float a){
                    const float PI = 3.1415926535f; const float TWO_PI = 6.283185307f;
                    if (a >  PI) a -= TWO_PI; else if (a < -PI) a += TWO_PI; return a;
                };
                float e_yaw = wrap_pi(yaw_rad - yaw_ref_rad);
                // use bias-corrected gyro rate from estimator update
                float yaw_rate = latest_gyro_z; // rad/s
                // rate PI (+ optional small angle P) for zero steady-state yaw-rate
                // integral update with anti-windup limit
                yaw_i_accum_us += ki_yaw_us_per_rad_s * (-yaw_rate) * static_cast<float>(dt_sec);
                if (yaw_i_accum_us > yaw_i_limit_us) yaw_i_accum_us = yaw_i_limit_us; else if (yaw_i_accum_us < -yaw_i_limit_us) yaw_i_accum_us = -yaw_i_limit_us;
                float c_yaw_us = kp_yaw_us_per_rad * (-e_yaw) + kd_yaw_us_per_rad_s * (-yaw_rate) + yaw_i_accum_us;
                c_yaw_us *= YAW_RATE_SIGN; // quick polarity toggle
                // wider clamp and earlier engagement
                if (c_yaw_us > 300.0f) c_yaw_us = 300.0f; else if (c_yaw_us < -300.0f) c_yaw_us = -300.0f;
                bool yaw_active = (elapsed_ms >= (TAKEOFF_MS + 500));
                if (!yaw_active) yaw_i_accum_us = 0.0f; // avoid windup before engagement
                float cy = yaw_active ? (Y_SIGN * c_yaw_us) : 0.0f;
                last_cy_us = cy;
                // apply yaw mix: FR/RL +, FL/RR - (assumes rotor directions CW at FR/RL)
                r1f += cy; // FR
                r2f += cy; // RL
                r3f -= cy; // FL
                r4f -= cy; // RR

                // mixer normalization: keep average at HOVER_US to prevent thrust drift
                {
                    float avg = 0.25f * (r1f + r2f + r3f + r4f);
                    float delta = avg - static_cast<float>(HOVER_US);
                    r1f -= delta; r2f -= delta; r3f -= delta; r4f -= delta;
                }

                // periodic auto-tune (every ~1s window)
                if (ENABLE_AUTO_TUNE && elapsed_ms >= TAKEOFF_MS && (now - last_autotune_tp) >= std::chrono::seconds(1)) {
                    float ex_avg = e_count > 0 ? static_cast<float>(ex_sum / e_count) : 0.0f;
                    float ey_avg = e_count > 0 ? static_cast<float>(ey_sum / e_count) : 0.0f;

                    bool adjusted = false;
                    // steady-state bias: raise Ki and I-limit (per-axis)
                    if (std::fabs(ex_avg) > EX_BIAS_TH) {
                        if (roll_ki < R_KI_MAX) { roll_ki = std::min(R_KI_MAX, roll_ki + KI_STEP_R); adjusted = true; }
                        if (roll_i_lim < R_ILIM_MAX) { roll_i_lim = std::min(R_ILIM_MAX, roll_i_lim + ILIM_STEP); adjusted = true; }
                    }
                    if (std::fabs(ey_avg) > EY_BIAS_TH) {
                        if (pitch_ki < P_KI_MAX) { pitch_ki = std::min(P_KI_MAX, pitch_ki + KI_STEP_P); adjusted = true; }
                        if (pitch_i_lim < P_ILIM_MAX) { pitch_i_lim = std::min(P_ILIM_MAX, pitch_i_lim + ILIM_STEP); adjusted = true; }
                    }
                    // oscillation: prefer increasing kd a bit, if excessive flips also lower kp slightly
                    if (roll_sign_flips > FLIP_TH) {
                        if (kd_roll_us_per_rad_s < KD_MAX) { kd_roll_us_per_rad_s = std::min(KD_MAX, kd_roll_us_per_rad_s + KD_STEP); adjusted = true; }
                        if (roll_kp > KP_MIN) { roll_kp = std::max(KP_MIN, roll_kp - KP_STEP); adjusted = true; }
                    }
                    if (pitch_sign_flips > FLIP_TH) {
                        if (kd_pitch_us_per_rad_s < KD_MAX) { kd_pitch_us_per_rad_s = std::min(KD_MAX, kd_pitch_us_per_rad_s + KD_STEP); adjusted = true; }
                        if (pitch_kp > KP_MIN) { pitch_kp = std::max(KP_MIN, pitch_kp - KP_STEP); adjusted = true; }
                    }

                    if (adjusted) {
                        pid_roll.configure(roll_kp, roll_ki, roll_i_lim);
                        pid_pitch.configure(pitch_kp, pitch_ki, pitch_i_lim);
                        std::ostringstream aos; aos.setf(std::ios::fixed); aos.precision(2);
                        aos << "autotune: ex_avg=" << ex_avg << " ey_avg=" << ey_avg
                            << " | R(kp,ki,ilim,kd)= " << roll_kp << "," << roll_ki << "," << roll_i_lim << "," << kd_roll_us_per_rad_s
                            << " P(kp,ki,ilim,kd)= " << pitch_kp << "," << pitch_ki << "," << pitch_i_lim << "," << kd_pitch_us_per_rad_s
                            << " flips R/P= " << roll_sign_flips << "/" << pitch_sign_flips;
                        logger.logText(aos.str());
                    }

                    // reset window
                    ex_sum = ey_sum = 0.0; e_count = 0; roll_sign_flips = pitch_sign_flips = 0;
                    last_autotune_tp = now;
                }
                // clamp 1000..2000 (floats)
                auto clampf = [](float v){ if (v < 1000.0f) return 1000.0f; if (v > 2000.0f) return 2000.0f; return v; };
                r1f = clampf(r1f); r2f = clampf(r2f); r3f = clampf(r3f); r4f = clampf(r4f);
                // error-diffusion rounding (dither) to minimize quantization bias
                static float frac1=0.0f, frac2=0.0f, frac3=0.0f, frac4=0.0f;
                float v1 = r1f + frac1; float v2 = r2f + frac2; float v3 = r3f + frac3; float v4 = r4f + frac4;
                int i1 = static_cast<int>(std::floor(v1 + 0.5f));
                int i2 = static_cast<int>(std::floor(v2 + 0.5f));
                int i3 = static_cast<int>(std::floor(v3 + 0.5f));
                int i4 = static_cast<int>(std::floor(v4 + 0.5f));
                frac1 = v1 - static_cast<float>(i1);
                frac2 = v2 - static_cast<float>(i2);
                frac3 = v3 - static_cast<float>(i3);
                frac4 = v4 - static_cast<float>(i4);
                // per-rotor slew limit (smooth output changes)
                {
                    static int prev1 = HOVER_US, prev2 = HOVER_US, prev3 = HOVER_US, prev4 = HOVER_US;
                    auto clampi = [](int dv, int lim){ if (dv > lim) return lim; if (dv < -lim) return -lim; return dv; };
                    const int SLEW_US = 4;
                    int d1 = clampi(i1 - prev1, SLEW_US);
                    int d2 = clampi(i2 - prev2, SLEW_US);
                    int d3 = clampi(i3 - prev3, SLEW_US);
                    int d4 = clampi(i4 - prev4, SLEW_US);
                    prev1 += d1; prev2 += d2; prev3 += d3; prev4 += d4;
                    i1 = prev1; i2 = prev2; i3 = prev3; i4 = prev4;
                }
                // final assign
                r1 = static_cast<uint16_t>(i1);
                r2 = static_cast<uint16_t>(i2);
                r3 = static_cast<uint16_t>(i3);
                r4 = static_cast<uint16_t>(i4);

                // anti-windup: if any motor saturates, clear integrators
                bool saturated = (i1 <= 1002 || i1 >= 1998 || i2 <= 1002 || i2 >= 1998 || i3 <= 1002 || i3 >= 1998 || i4 <= 1002 || i4 >= 1998);
                if (saturated) { pid_roll.reset(); pid_pitch.reset(); yaw_i_accum_us = 0.0f; }

                // 50 Hz debug PDI terms and rate/mix snapshot (only when DIAG_MODE)
            if (DIAG_MODE && (now - last_debug) >= DEBUG_INTERVAL) {
                    std::ostringstream os2; os2.setf(std::ios::fixed); os2.precision(2);
                    float mix_avg = 0.25f * (r1f + r2f + r3f + r4f);
                    // compute Euler for estimator and GT
                    float est_r=0, est_p=0, est_y=0; AttitudeUtils::computeEulerDeg(qw,qx,qy,qz, est_r, est_p, est_y);
                    float gt_r=0, gt_p=0, gt_y=0;
                    if (have_telem) {
                        AttitudeUtils::computeEulerDeg(
                            static_cast<float>(last_telem.quat[0]),
                            static_cast<float>(last_telem.quat[1]),
                            static_cast<float>(last_telem.quat[2]),
                            static_cast<float>(last_telem.quat[3]),
                            gt_r, gt_p, gt_y);
                    }
                    // quaternion delta angle between EST and GT
                    float qgtw = have_telem ? static_cast<float>(last_telem.quat[0]) : 1.0f;
                    float qgtx = have_telem ? static_cast<float>(last_telem.quat[1]) : 0.0f;
                    float qgty = have_telem ? static_cast<float>(last_telem.quat[2]) : 0.0f;
                    float qgtz = have_telem ? static_cast<float>(last_telem.quat[3]) : 0.0f;
                    float dot = std::fabs(qw*qgtw + qx*qgtx + qy*qgty + qz*qgtz);
                    if (dot > 1.0f) dot = 1.0f; else if (dot < -1.0f) dot = -1.0f;
                    float qerr_deg = 2.0f * std::acos(dot) * 57.2957795f;
                    // error magnitude ratio (guard small denom)
                    auto fabsf_ = [](float v){ return v >= 0 ? v : -v; };
                    float ratio_ex = fabsf_(ex_gt) / (fabsf_(ex_est) + 1e-6f);
                    float ratio_ey = fabsf_(ey_gt) / (fabsf_(ey_est) + 1e-6f);
                    os2 << "diag est[r p y]= " << est_r << " " << est_p << " " << est_y
                        << " gt[r p y]= " << gt_r << " " << gt_p << " " << gt_y
                        << " dEuler= " << (gt_r - est_r) << " " << (gt_p - est_p) << " " << (gt_y - est_y)
                        << " qerr_deg= " << qerr_deg
                        << " err(est ex ey)= " << ex_est << " " << ey_est
                        << " err(gt  ex ey)= " << ex_gt << " " << ey_gt
                        << " ratio(ex/ey)= " << ratio_ex << "/" << ratio_ey
                        << " gains[kp ki]= " << est_kp_cur << " " << est_ki_cur
                        << " acc_norm= " << last_acc_norm
                        << " src= " << (gt_blend > 0.5f ? "GT" : "EST") << "(w=" << gt_blend << ")"
                        << " cp/cr(us)= " << cp << "/" << cr
                        << " mix_avg= " << mix_avg;
                    logger.logText(os2.str());
                    last_debug = now;
                }
            }
            writer.write(r1, r2, r3, r4);

            // drain telemetry and keep estimator fresh for logging
            io.handle().consume_telem(last_seq, [&](const XSimTelemetry& d){
                last_telem = d; have_telem = true;
                float gyro[3] = { static_cast<float>(d.gyro[0]), static_cast<float>(d.gyro[1]), static_cast<float>(d.gyro[2]) };
                float acc[3]  = { static_cast<float>(d.acc[0]),  static_cast<float>(d.acc[1]),  static_cast<float>(d.acc[2]) };
                float dt = 0.001f;
                if (last_ts_ns != 0) {
                    long long dtns = d.timestamp - last_ts_ns;
                    if (dtns > 0 && dtns < 10000000LL) dt = static_cast<float>(dtns * 1e-9f);
                }
                last_ts_ns = d.timestamp;
                if (dt < 0.0005f) dt = 0.0005f; else if (dt > 0.0025f) dt = 0.0025f;
                float an = std::sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
                float gyro_abs = std::fabs(gyro[0]) + std::fabs(gyro[1]) + std::fabs(gyro[2]);
                if (gyro_abs < 0.15f && std::fabs(an - 9.80665f) < 1.0f) {
                    const float beta = 0.001f;
                    gyro_bias[0] = (1.0f - beta) * gyro_bias[0] + beta * gyro[0];
                    gyro_bias[1] = (1.0f - beta) * gyro_bias[1] + beta * gyro[1];
                    gyro_bias[2] = (1.0f - beta) * gyro_bias[2] + beta * gyro[2];
                }
                float gyro_corr[3] = { gyro[0] - gyro_bias[0], gyro[1] - gyro_bias[1], gyro[2] - gyro_bias[2] };
                estimator.update(gyro_corr, acc, dt);
            });
                if ((now - last_summary) >= SUMMARY_INTERVAL) {
                last_summary = now;
                std::ostringstream os; os.setf(std::ios::fixed); os.precision(2);
                float qw=1, qx=0, qy=0, qz=0; estimator.getQuaternion(qw,qx,qy,qz);
                float roll_deg=0, pitch_deg=0, yaw_deg=0;
                AttitudeUtils::computeEulerDeg(qw,qx,qy,qz, roll_deg, pitch_deg, yaw_deg);
                // also compute ground-truth from telemetry quat for comparison
                float gt_r=0, gt_p=0, gt_y=0;
                if (have_telem) {
                    AttitudeUtils::computeEulerDeg(
                        static_cast<float>(last_telem.quat[0]),
                        static_cast<float>(last_telem.quat[1]),
                        static_cast<float>(last_telem.quat[2]),
                        static_cast<float>(last_telem.quat[3]),
                        gt_r, gt_p, gt_y);
                }
                if (elapsed_ms < TAKEOFF_MS) {
                    os << "takeoff_" << TAKEOFF_US << " mix[FR RL FL RR]= " << r1 << " " << r2 << " " << r3 << " " << r4
                       << " est[r p y]= " << roll_deg << " " << pitch_deg << " " << yaw_deg
                       << " gt[r p y]= " << gt_r << " " << gt_p << " " << gt_y;
                } else {
                    os << g_summary_prefix << " fr+" << BIAS_FR << " rl+" << BIAS_RL << " fl+" << BIAS_FL << " rr+" << BIAS_RR
                       << " mix[FR RL FL RR]= " << r1 << " " << r2 << " " << r3 << " " << r4
                       << " est[r p y]= " << roll_deg << " " << pitch_deg << " " << yaw_deg
                       << " gt[r p y]= " << gt_r << " " << gt_p << " " << gt_y
                       << " yaw[rate,cy]= " << latest_gyro_z << "," << last_cy_us
                       << " cp/cr(us)= " << last_cp_us << "/" << last_cr_us;
                }
                logger.logText(os.str());
            }
            next_tp += period;
            if (max_duration_ms > 0 && elapsed_ms >= max_duration_ms) {
                break; // end scenario after specified duration
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

static void run_all(XSimIo& io, LogHelper& logger)
{
    // Per scenario duration (ms): include 7s takeoff + 3.8s trim + 90s window margin
    const int SCEN_MS = 130000; // ~130 s
    // S1: Hover-bias
    g_summary_prefix = "hover_bias";
    scenario_takeoff_hover_bias(io, logger, SCEN_MS);
    // S2: Gyro Bias (reuse same loop; tag only)
    g_summary_prefix = "gyro_bias";
    scenario_takeoff_hover_bias(io, logger, SCEN_MS);
    // S3: Dynamic Accel (tag only)
    g_summary_prefix = "dyn_accel";
    scenario_takeoff_hover_bias(io, logger, SCEN_MS);
    // S4: Aggressive Tilt (tag only)
    g_summary_prefix = "agg_tilt";
    scenario_takeoff_hover_bias(io, logger, SCEN_MS);
}

int main()
{
	std::cout << "test_client" << std::endl;
	timeBeginPeriod(1);

	// Elevate control loop thread priority on Windows (Controller ~ High)
	#ifdef _WIN32
		SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
	#endif

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

    // Run all scenarios in a single execution (batch mode)
    run_all(io, logger);
	timeEndPeriod(1);
	return 0;
}


