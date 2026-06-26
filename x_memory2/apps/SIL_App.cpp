#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <windows.h>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <cstring>
#include <string>

#include "core/Driver_IF.hpp"
#include "core/IMU_IF.hpp"
#include "core/ACT_IF.hpp"
#include "core/GNSS_IF.hpp"
#include "core/BARO_IF.hpp"
#include "core/PID.hpp"

#include "drivers/shm/SHM_Driver.hpp"
#include "drivers/rpc/RPC_Driver.hpp"
#include "modules/imu/IMU_SimSHM.hpp"
#include "modules/actuator/ACT_SimSHM.hpp"
#include "modules/gnss/GNSS_SimSHM.hpp"
#include "modules/baro/BARO_SimSHM.hpp"
#include "modules/estimator/EST_EKF24.hpp"
#include "../shm/lib/AttitudeUtils.hpp"
#include "core/PerfStats.hpp"
#include <fstream>
#include <atomic>
#include <mutex>
#include <algorithm>

#pragma comment(lib, "winmm.lib")
#pragma comment(lib, "ws2_32.lib")

std::ofstream log_file;
std::atomic<bool> should_exit(false);

// Per-sample latency/jitter record (buffered in memory, dumped to perf_log.csv at exit
// so that disk I/O does not perturb the timing being measured)
struct PerfSample {
    uint64_t sim_ts_ns;       // simulator IMU timestamp
    double   wall_us;         // wall-clock time since loop start
    double   read_us;         // imu.read() call latency
    double   interarrival_us; // wall-clock gap to previous processed sample
    double   jitter_us;       // interarrival - target period
    double   sim_dt_us;       // sim-time gap (sensor side)
};

// Thread-safe command variables
std::mutex g_cmd_mutex;
char g_cmd_axis = '\0';
float g_cmd_value = 0.0f;
float g_cmd_duration = 0.0f;
bool g_new_cmd_received = false;

// UDP Command Server thread function
void command_listener_thread() {
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);

    SOCKET recv_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (recv_socket == INVALID_SOCKET) {
        std::cerr << "[UDP Server] Socket creation failed. Error: " << WSAGetLastError() << std::endl;
        return;
    }

    sockaddr_in recv_addr;
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_port = htons(5005);
    recv_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(recv_socket, (SOCKADDR*)&recv_addr, sizeof(recv_addr)) == SOCKET_ERROR) {
        std::cerr << "[UDP Server] Bind failed. Error: " << WSAGetLastError() << std::endl;
        closesocket(recv_socket);
        return;
    }

    printf("[UDP Server] Command listener thread started on UDP port 5005...\n");

    char recv_buf[256];
    sockaddr_in sender_addr;
    int sender_addr_size = sizeof(sender_addr);

    // Make socket non-blocking
    u_long mode = 1;
    ioctlsocket(recv_socket, FIONBIO, &mode);

    while (!should_exit) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(recv_socket, &fds);
        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 50000; // 50ms check interval

        int sel = select(0, &fds, nullptr, nullptr, &tv);
        if (sel > 0 && FD_ISSET(recv_socket, &fds)) {
            int bytes_recvd = recvfrom(recv_socket, recv_buf, sizeof(recv_buf) - 1, 0,
                                       (SOCKADDR*)&sender_addr, &sender_addr_size);
            if (bytes_recvd > 0) {
                recv_buf[bytes_recvd] = '\0';
                
                char axis = '\0';
                float val = 0.0f;
                float dur = 0.0f;
                if (sscanf(recv_buf, " %c %f %f", &axis, &val, &dur) == 3) {
                    if (axis == 'r' || axis == 'p' || axis == 'y' || axis == 'a') {
                        std::lock_guard<std::mutex> lock(g_cmd_mutex);
                        g_cmd_axis = axis;
                        g_cmd_value = val;
                        g_cmd_duration = dur;
                        g_new_cmd_received = true;
                        if (axis == 'a') {
                            printf("\n[UDP Server] Received command: Axis=%c, Val=%.2f m, Dur=%.2f sec\n", axis, val, dur);
                        } else {
                            printf("\n[UDP Server] Received command: Axis=%c, Val=%.2f deg, Dur=%.2f sec\n", axis, val, dur);
                        }
                        fflush(stdout);
                    }
                }
            }
        }
    }

    closesocket(recv_socket);
    printf("[UDP Server] Command listener thread stopped.\n");
}

BOOL WINAPI CtrlHandler(DWORD fdwCtrlType) {
    if (fdwCtrlType == CTRL_C_EVENT) {
        printf("\n[Ctrl+C] Stopping autopilot loop and flushing logs...\n");
        should_exit = true;
        return TRUE;
    }
    return FALSE;
}

int main(int argc, char* argv[]) {
    timeBeginPeriod(1);

    double target_hz = 8000.0;
    bool is_takeoff_active = false;
    double target_altitude = 0.0;
    float current_target_alt = 0.0f;

    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "takeoff" && i + 1 < argc) {
            target_altitude = std::stod(argv[i + 1]);
            is_takeoff_active = true;
            i++; // skip next argument
        } else {
            try {
                target_hz = std::stod(arg);
            } catch (...) {
                // Ignore invalid parameters
            }
        }
    }

    // --- [Option 2] SHM Mode ---
    printf("[SIL_App] Mode: SHM\n");
    if (is_takeoff_active) {
        printf("[SIL_App] Command: Takeoff to %.2f meters\n", target_altitude);
    }
    
    // Frequency-tagged log filenames so 1k/4k/8k runs don't overwrite each other
    int hz_tag = (int)(target_hz + 0.5);
    std::string flight_log_name = "flight_log_" + std::to_string(hz_tag) + "hz.csv";
    std::string perf_log_name = "perf_log_" + std::to_string(hz_tag) + "hz.csv";

    SHM_Driver driver("AirSimXsim");
    IMU_SimSHM imu(driver);
    GNSS_SimSHM gnss(driver);
    BARO_SimSHM baro(driver);
    ACT_SimSHM act(driver);
    auto& target_driver = driver;

    EST_EKF24 ekf;

    SetConsoleCtrlHandler(CtrlHandler, TRUE);
    std::thread cmd_thread(command_listener_thread);

    // Open CSV flight log
    log_file.open(flight_log_name);
    if (log_file.is_open()) {
        log_file << "timestamp_ns,dt,raw_baro_alt,gps_alt,ekf_alt,ekf_vel_z,pid_target,pid_out_throttle\n";
    }

    // Reset simulation environment via RPC before starting/connecting SHM
    {
        RPC_Driver rpc_driver;
        if (rpc_driver.connect()) {
            printf("[SIL_App] Connected to AirSim RPC. Resetting simulation...\n");
            try {
                rpc_driver.get_client()->reset();
                // Wait a moment for simulator state to stabilize after reset
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                
                // Re-enable API control and arm the vehicle (reset() disables them by default)
                printf("[SIL_App] Re-enabling API control and arming vehicle...\n");
                rpc_driver.get_client()->enableApiControl(true);
                rpc_driver.get_client()->armDisarm(true);
            } catch (const std::exception& e) {
                std::cerr << "[SIL_App] Warning: RPC reset/init failed: " << e.what() << std::endl;
            }
            rpc_driver.close();
        } else {
            printf("[SIL_App] Warning: Could not connect to AirSim RPC (port 41451). Reset skipped.\n");
        }
    }

    if (!target_driver.connect()) {
        std::cerr << "[SIL_App] Failed to connect!" << std::endl;
        if (log_file.is_open()) log_file.close();
        return 1;
    }

    imu.init();
    gnss.init();
    baro.init();
    ekf.init();
    act.init(); // This will enable API control and arm the drone
    current_target_alt = (float)target_altitude;

    // Setup PID Controllers
    PID pid_alt;
    pid_alt.set_gains(0.18f, 0.04f, 0.10f, 0.0f);
    pid_alt.set_limits(0.20f, -0.25f, 0.25f); // Limit integration and total correction output
    pid_alt.set_d_filter_hz(10.0f);

    PID pid_roll;
    pid_roll.set_gains(0.08f, 0.02f, 0.005f, 0.0f);
    pid_roll.set_limits(0.10f, -0.15f, 0.15f);
    pid_roll.set_d_filter_hz(20.0f);

    PID pid_pitch;
    pid_pitch.set_gains(0.08f, 0.02f, 0.005f, 0.0f);
    pid_pitch.set_limits(0.10f, -0.15f, 0.15f);
    pid_pitch.set_d_filter_hz(20.0f);

    PID pid_yaw;
    pid_yaw.set_gains(0.15f, 0.05f, 0.0f, 0.0f);
    pid_yaw.set_limits(0.10f, -0.15f, 0.15f);

    // Target intervals in Nanoseconds
    const uint64_t IMU_TARGET_NS = (uint64_t)(1000000000.0 / target_hz);
    const uint64_t PWM_TARGET_NS = 2500000;   // 400 Hz (2.5ms)

    IMUData imu_data;
    PWMData pwm_data;
    pwm_data.pwm_values = { 0.5f, 0.5f, 0.5f, 0.5f }; // Default Neutral

    uint64_t last_imu_ts = 0;
    uint64_t last_pwm_send_ts = 0;

    bool yaw_ref_set = false;
    float yaw_ref_rad = 0.0f;

    // Command targets from UDP server
    uint64_t cmd_end_time_ns = 0;
    char cmd_active_axis = '\0';
    float cmd_active_value = 0.0f;
    const float hover_throttle = 0.59f; // Base throttle for multirotor to hover (approx 1590us)
    float current_target_roll = 0.0f;
    float current_target_pitch = 0.0f;
    float current_target_yaw = 0.0f;

    // Statistics via PerfStats class
    PerfStats stats;
    stats.setTargetHz(target_hz);

    // --- Latency/Jitter perf logging (dumped to perf_log.csv at exit) ---
    const double IMU_TARGET_US = 1000000.0 / target_hz;
    std::vector<PerfSample> perf_samples;
    perf_samples.reserve(2000000); // ~250s @8kHz; recording stops once capacity is reached
    auto loop_start_wall = std::chrono::steady_clock::now();
    auto last_proc_wall = loop_start_wall;
    bool perf_first = true;

    std::cout << "[SIL_App] Loop starting..." << std::endl;
    std::cout << "   Hist (Delay%): [0]<5%, [1]<10%, [2]<15%, [3]<20%, [4]<30%, [5]>=30%" << std::endl;

    // --- Fixed-rate loop scheduler: coarse sleep_until then spin to hit each deadline ---
    const auto tick_period = std::chrono::nanoseconds((long long)(1000000000.0 / target_hz));
    auto next_deadline = std::chrono::steady_clock::now() + tick_period;

    while (!should_exit) {
        // Coarse sleep to release CPU (only effective when period > ~1ms on Windows),
        // then busy-spin the remainder for precise deadline hit (required for 4k/8k).
        auto coarse_wake = next_deadline - std::chrono::microseconds(1000);
        if (coarse_wake > std::chrono::steady_clock::now())
            std::this_thread::sleep_until(coarse_wake);
        while (std::chrono::steady_clock::now() < next_deadline) { YieldProcessor(); }

        auto t_wake = std::chrono::steady_clock::now();
        double tick_jitter_us = std::chrono::duration<double, std::micro>(t_wake - next_deadline).count();
        next_deadline += tick_period;
        // If we fell badly behind (e.g. after an OS hiccup), realign instead of bursting.
        if (next_deadline < t_wake) next_deadline = t_wake + tick_period;

        auto t_imu_start = std::chrono::steady_clock::now();
        bool success = imu.read(imu_data);
        auto t_imu_end = std::chrono::steady_clock::now();

        if (success) {
            if (last_imu_ts == 0) last_imu_ts = imu_data.timestamp_ns;

            uint64_t diff = imu_data.timestamp_ns - last_imu_ts;

            {   // process every scheduled tick (rate is set by the scheduler, no sim-time gate)
                stats.recordImuCall(std::chrono::duration<double, std::milli>(t_imu_end - t_imu_start).count());
                stats.recordValidImu(diff / 1000000.0);

                float dt = 1.0f / (float)target_hz;   // nominal fixed loop period
                last_imu_ts = imu_data.timestamp_ns;

                // --- Latency/Jitter sample: deadline miss (tick_jitter) is the true scheduling jitter ---
                double perf_read_us = std::chrono::duration<double, std::micro>(t_imu_end - t_imu_start).count();
                double perf_wall_us = std::chrono::duration<double, std::micro>(t_wake - loop_start_wall).count();
                double perf_inter_us = perf_first ? 0.0 : std::chrono::duration<double, std::micro>(t_wake - last_proc_wall).count();
                double perf_jit_us = tick_jitter_us;
                last_proc_wall = t_wake;
                perf_first = false;
                if (perf_samples.size() < perf_samples.capacity()) {
                    perf_samples.push_back({ imu_data.timestamp_ns, perf_wall_us, perf_read_us,
                                             perf_inter_us, perf_jit_us, (double)diff * 1e-3 });
                }

                // Propagate EKF state using IMU
                ekf.update_imu(imu_data);

                // --- Read GNSS and Baro (Common Sensor API Layer) ---
                GNSSData gnss_data;
                BAROData baro_data;
                bool has_gnss = gnss.read(gnss_data);
                bool has_baro = baro.read(baro_data);

                // Fuse GNSS and Barometer updates in EKF
                if (has_gnss) {
                    ekf.update_gnss(gnss_data);
                }
                if (has_baro) {
                    ekf.update_baro(baro_data);
                }
                if (imu_data.mag[0] != 0.0 || imu_data.mag[1] != 0.0 || imu_data.mag[2] != 0.0) {
                    ekf.update_mag(imu_data.mag);
                }

                EstimatedState est;
                bool has_est = ekf.get_estimated_state(est);

                // Check for new UDP command
                char active_axis = '\0';
                float active_value = 0.0f;
                float active_duration = 0.0f;
                bool got_cmd = false;
                {
                    std::lock_guard<std::mutex> lock(g_cmd_mutex);
                    if (g_new_cmd_received) {
                        active_axis = g_cmd_axis;
                        active_value = g_cmd_value;
                        active_duration = g_cmd_duration;
                        g_new_cmd_received = false;
                        got_cmd = true;
                    }
                }

                if (got_cmd) {
                    cmd_end_time_ns = imu_data.timestamp_ns + static_cast<uint64_t>(active_duration * 1e9);
                    cmd_active_axis = active_axis;
                    if (active_axis == 'a') {
                        cmd_active_value = active_value; // Meters
                        target_altitude = active_value;  // Update target altitude permanently
                        is_takeoff_active = true;         // Ensure autopilot/takeoff loop is active
                    } else {
                        cmd_active_value = active_value * 0.01745329f; // Convert degrees to radians
                    }
                }
                
                float current_altitude = has_baro ? (float)baro_data.altitude : 0.0f;

                // Log state to CSV
                if (log_file.is_open()) {
                    float e_alt = has_est ? -est.pos_ned[2] : 0.0f;
                    float e_vel_z = has_est ? -est.vel_ned[2] : 0.0f;
                    float g_alt = has_gnss ? -gnss_data.pos_ned[2] : 0.0f;

                    log_file << imu_data.timestamp_ns << ","
                             << dt << ","
                             << current_altitude << ","
                             << g_alt << ","
                             << e_alt << ","
                             << e_vel_z << ","
                             << (float)target_altitude << ","
                             << pwm_data.pwm_values[0] << "\n";

                    static int log_counter = 0;
                    if (++log_counter >= 1000) {
                        log_file.flush();
                        log_counter = 0;
                    }
                }

                // Autopilot control logic
                if (is_takeoff_active && dt > 0.0f) {
                    // Get current attitude from EKF
                    float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
                    if (has_est) {
                        qw = est.quat[0];
                        qx = est.quat[1];
                        qy = est.quat[2];
                        qz = est.quat[3];
                    }

                    float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
                    AttitudeUtils::computeEulerDeg(qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg);

                    float roll_rad = roll_deg * 0.01745329f;
                    float pitch_rad = pitch_deg * 0.01745329f;
                    float yaw_rad = yaw_deg * 0.01745329f;

                    if (!yaw_ref_set) {
                        yaw_ref_rad = yaw_rad;
                        yaw_ref_set = true;
                    }

                    // Default setpoints
                    current_target_roll = 0.0f;
                    current_target_pitch = 0.0f;
                    current_target_yaw = yaw_ref_rad;
                    current_target_alt = (float)target_altitude;

                    // Apply command override if within duration
                    if (imu_data.timestamp_ns < cmd_end_time_ns) {
                        if (cmd_active_axis == 'r') {
                            current_target_roll = cmd_active_value;
                        } else if (cmd_active_axis == 'p') {
                            current_target_pitch = cmd_active_value;
                        } else if (cmd_active_axis == 'y') {
                            current_target_yaw = yaw_ref_rad + cmd_active_value;
                        } else if (cmd_active_axis == 'a') {
                            current_target_alt = cmd_active_value;
                        }
                    }

                    // --- 1. Altitude Control Loop ---
                    // Use EKF estimated altitude if healthy, otherwise fall back to raw barometer
                    float feedback_alt = has_est ? -est.pos_ned[2] : current_altitude;
                    float alt_output = pid_alt.compute(current_target_alt, feedback_alt, dt);
                    float throttle = hover_throttle + alt_output;

                    float target_roll = current_target_roll;
                    float target_pitch = current_target_pitch;
                    float target_yaw = current_target_yaw;

                    // --- 2. Attitude Control Loop ---
                    // Outer loop: Angle errors (target is level roll/pitch, holding yaw)
                    float roll_err = target_roll - roll_rad;
                    float pitch_err = target_pitch - pitch_rad;
                    
                    auto wrap_pi = [](float a) {
                        const float PI_VAL = 3.1415926535f;
                        const float TWO_PI_VAL = 6.283185307f;
                        if (a > PI_VAL) a -= TWO_PI_VAL;
                        else if (a < -PI_VAL) a += TWO_PI_VAL;
                        return a;
                    };
                    float yaw_err = wrap_pi(target_yaw - yaw_rad);

                    float target_roll_rate = 4.5f * roll_err;
                    float target_pitch_rate = 5.0f * pitch_err;
                    float target_yaw_rate = 1.5f * yaw_err;

                    // Inner loop: Rate PID
                    float cr = pid_roll.compute(target_roll_rate, (float)imu_data.gyro[0], dt);
                    float cp = pid_pitch.compute(target_pitch_rate, (float)imu_data.gyro[1], dt);
                    float cy = pid_yaw.compute(target_yaw_rate, (float)imu_data.gyro[2], dt);

                    // --- 3. Actuator Mixer ---
                    float fr = throttle + cp - cr + cy; // FR
                    float fl = throttle + cp + cr - cy; // FL
                    float rl = throttle - cp + cr + cy; // RL
                    float rr = throttle - cp - cr - cy; // RR

                    // Clamp to safe limits (0.0 to 1.0)
                    auto clamp = [](float v) {
                        return (v < 0.0f) ? 0.0f : ((v > 1.0f) ? 1.0f : v);
                    };
                    
                    pwm_data.pwm_values = {
                        clamp(fr), // FR [0]
                        clamp(rl), // RL [1]
                        clamp(fl), // FL [2]
                        clamp(rr)  // RR [3]
                    };
                }

                static int print_counter = 0;
                print_counter++;
                if (print_counter >= 8000) { // ~1 second interval at 8kHz
                    print_counter = 0;

                    // Compute Euler angles for EKF
                    float e_r = 0.0f, e_p = 0.0f, e_y = 0.0f;
                    if (has_est) {
                        AttitudeUtils::computeEulerDeg(est.quat[0], est.quat[1], est.quat[2], est.quat[3], e_r, e_p, e_y);
                    }

                    if (is_takeoff_active) {
                        printf("[Autopilot] Alt (Target/Curr/EKF): %.2f / %.2f / %.2f | Throttle: %.3f\n", 
                               current_target_alt, current_altitude, has_est ? est.pos_ned[2] * -1.0f : 0.0f, pwm_data.pwm_values[0]);
                    }

                    if (has_est) {
                        printf("[EKF Pos  ] GPS: [%6.2f, %6.2f, %6.2f] | EKF: [%6.2f, %6.2f, %6.2f]\n",
                               gnss_data.pos_ned[0], gnss_data.pos_ned[1], gnss_data.pos_ned[2],
                               est.pos_ned[0], est.pos_ned[1], est.pos_ned[2]);
                        printf("[EKF Vel  ] GPS: [%6.2f, %6.2f, %6.2f] | EKF: [%6.2f, %6.2f, %6.2f]\n",
                               gnss_data.vel_ned[0], gnss_data.vel_ned[1], gnss_data.vel_ned[2],
                               est.vel_ned[0], est.vel_ned[1], est.vel_ned[2]);
                        printf("[EKF Att  ] EKF [R/P/Y]: [%5.1f, %5.1f, %5.1f]\n",
                               e_r, e_p, e_y);
                        printf("[TargetAtt ] R/P/Y: [%5.1f, %5.1f, %5.1f]\n",
                               current_target_roll * 57.2957795f, current_target_pitch * 57.2957795f, current_target_yaw * 57.2957795f);
                        printf("[EKF Bias ] Gyro: [%7.4f, %7.4f, %7.4f] | Accel: [%6.3f, %6.3f, %6.3f]\n",
                               est.gyro_bias[0], est.gyro_bias[1], est.gyro_bias[2],
                               est.accel_bias[0], est.accel_bias[1], est.accel_bias[2]);
                        printf("-------------------------------------------------------------------------\n");
                        fflush(stdout);
                    }
                }
            }
        }

        // --- 2. Actuator Writing (400Hz) ---
        // Use IMU timestamp for synchronization
        if (last_pwm_send_ts == 0) last_pwm_send_ts = imu_data.timestamp_ns;

        if (imu_data.timestamp_ns - last_pwm_send_ts >= PWM_TARGET_NS) {
            auto t_pwm_start = std::chrono::steady_clock::now();
            act.write(pwm_data);
            auto t_pwm_end = std::chrono::steady_clock::now();
            
            stats.recordPwmCall(std::chrono::duration<double, std::milli>(t_pwm_end - t_pwm_start).count());

            // Update using target interval to maintain 400Hz on average (3ms, 2ms, 3ms...)
            last_pwm_send_ts += PWM_TARGET_NS;
        }

        // --- 3. Reporting ---
        stats.updateAndReport();
    }

    if (cmd_thread.joinable()) {
        cmd_thread.join();
    }

    if (log_file.is_open()) {
        log_file.close();
        printf("[SIL_App] Log file saved to %s in the current working directory.\n", flight_log_name.c_str());
    }

    // --- Dump per-sample latency/jitter log + print summary ---
    {
        std::ofstream pf(perf_log_name);
        pf << "idx,sim_ts_ns,wall_us,read_us,interarrival_us,jitter_us,sim_dt_us\n";
        std::vector<double> inter;   // interarrival samples (skip first)
        std::vector<double> readv;   // read-latency samples (skip first)
        inter.reserve(perf_samples.size());
        readv.reserve(perf_samples.size());
        for (size_t i = 0; i < perf_samples.size(); ++i) {
            const PerfSample& s = perf_samples[i];
            pf << i << "," << s.sim_ts_ns << "," << s.wall_us << "," << s.read_us << ","
               << s.interarrival_us << "," << s.jitter_us << "," << s.sim_dt_us << "\n";
            if (i > 0) { inter.push_back(s.interarrival_us); readv.push_back(s.read_us); }
        }
        pf.close();

        if (!inter.empty()) {
            std::sort(inter.begin(), inter.end());
            std::sort(readv.begin(), readv.end());
            auto pct = [](const std::vector<double>& v, double p) {
                return v[(size_t)(p * (v.size() - 1))];
            };
            double sum = 0.0, sq = 0.0;
            for (double x : inter) { sum += x; sq += x * x; }
            double mean = sum / inter.size();
            double sd = std::sqrt(std::max(0.0, sq / inter.size() - mean * mean));
            double rsum = 0.0;
            for (double x : readv) rsum += x;
            double rmean = rsum / readv.size();

            printf("\n[Perf] target=%.2f us (%.0f Hz) | processed samples=%zu\n",
                   IMU_TARGET_US, target_hz, perf_samples.size());
            printf("[Perf] interarrival (us): mean=%.2f std=%.2f min=%.2f p50=%.2f p95=%.2f p99=%.2f p99.9=%.2f max=%.2f\n",
                   mean, sd, inter.front(), pct(inter, 0.50), pct(inter, 0.95),
                   pct(inter, 0.99), pct(inter, 0.999), inter.back());
            printf("[Perf] jitter vs target (us): mean=%.2f | read latency (us): mean=%.2f p95=%.2f max=%.2f\n",
                   mean - IMU_TARGET_US, rmean, pct(readv, 0.95), readv.back());
            printf("[Perf] %s saved (%zu rows).\n", perf_log_name.c_str(), perf_samples.size());
        }
    }

    return 0;
}
