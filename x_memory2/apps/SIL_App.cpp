#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <windows.h>
#include <direct.h>
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
    std::string log_dir = "d:/xlab/sim25/logs/silapp/";
    // Ensure folders exist
    _mkdir("d:/xlab");
    _mkdir("d:/xlab/sim25");
    _mkdir("d:/xlab/sim25/logs");
    _mkdir("d:/xlab/sim25/logs/silapp");

    std::string flight_log_name = log_dir + "flight_log_" + std::to_string(hz_tag) + "hz.csv";
    std::string perf_log_name = log_dir + "perf_log_" + std::to_string(hz_tag) + "hz.log";

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

    // Clear performance log
    {
        std::ofstream perf_f(perf_log_name, std::ios::out | std::ios::trunc);
        perf_f.close();
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
    uint64_t start_sim_ts = 0;
    uint64_t last_processed_sim_ts = 0;
    bool perf_first = true;

    // 1-second summary statistic variables
    uint32_t sil_count = 0;
    double sil_dt_sum = 0.0;
    double sil_dt_max = 0.0;
    double sil_dt_min = 1e18;
    double sil_dev_sum = 0.0;
    double sil_dev_max = 0.0;
    double sil_read_sum = 0.0;
    double sil_read_max = 0.0;
    double sil_read_min = 1e18;
    uint64_t last_report_sim_time = 0;
    auto last_report_wall_time = std::chrono::steady_clock::now();

    std::cout << "[SIL_App] Loop starting..." << std::endl;
    std::cout << "   Hist (Delay%): [0]<5%, [1]<10%, [2]<15%, [3]<20%, [4]<30%, [5]>=30%" << std::endl;

    while (!should_exit) {
        auto t_imu_start = std::chrono::steady_clock::now();
        bool success = false;

        // Busy-spin waiting for new telemetry packet
        while (!should_exit) {
            if (imu.read(imu_data)) {
                success = true;
                break;
            }
            YieldProcessor();
        }
        if (!success) continue;

        auto t_imu_end = std::chrono::steady_clock::now();

        if (start_sim_ts == 0) {
            start_sim_ts = imu_data.timestamp_ns;
            last_processed_sim_ts = imu_data.timestamp_ns;
        }

        uint64_t diff = imu_data.timestamp_ns - last_processed_sim_ts;
        last_processed_sim_ts = imu_data.timestamp_ns;

        {   // process every scheduled tick
            stats.recordImuCall(std::chrono::duration<double, std::milli>(t_imu_end - t_imu_start).count());
            stats.recordValidImu(diff / 1000000.0);

            float dt = 1.0f / (float)target_hz;   // nominal fixed loop period
            last_imu_ts = imu_data.timestamp_ns;

            // --- Latency/Jitter sample: measured in simulation time ---
            double perf_read_us = std::chrono::duration<double, std::micro>(t_imu_end - t_imu_start).count();
            double perf_inter_us = perf_first ? (1000000.0 / target_hz) : (double)diff / 1000.0;
            double perf_jit_us = std::abs(perf_inter_us - IMU_TARGET_US);
            perf_first = false;

            // Accumulate statistics
            sil_count++;
            sil_dt_sum += perf_inter_us;
            if (perf_inter_us > sil_dt_max) sil_dt_max = perf_inter_us;
            if (perf_inter_us < sil_dt_min) sil_dt_min = perf_inter_us;
            
            sil_dev_sum += perf_jit_us;
            if (perf_jit_us > sil_dev_max) sil_dev_max = perf_jit_us;

            sil_read_sum += perf_read_us;
            if (perf_read_us > sil_read_max) sil_read_max = perf_read_us;
            if (perf_read_us < sil_read_min) sil_read_min = perf_read_us;

            if (last_report_sim_time == 0) {
                last_report_sim_time = imu_data.timestamp_ns;
                last_report_wall_time = std::chrono::steady_clock::now();
            }

            uint64_t elapsed_ns = imu_data.timestamp_ns - last_report_sim_time;
            if (elapsed_ns >= 1000000000ULL) { // 1 second of simulation time
                double elapsed_sec = static_cast<double>(elapsed_ns) / 1e9;
                double phys_hz = static_cast<double>(sil_count) / elapsed_sec;
                
                auto now_wall_report = std::chrono::steady_clock::now();
                double elapsed_real_sec = std::chrono::duration<double>(now_wall_report - last_report_wall_time).count();
                double real_hz = elapsed_real_sec > 0.0 ? static_cast<double>(sil_count) / elapsed_real_sec : 0.0;
                
                try {
                    std::ofstream log_f(perf_log_name, std::ios::out | std::ios::app);
                    if (log_f.is_open()) {
                        time_t t = time(nullptr);
                        struct tm tm_info;
                        localtime_s(&tm_info, &t);
                        char time_str[32];
                        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &tm_info);

                        double err_pct = std::abs(phys_hz - target_hz) / target_hz * 100.0;
                        double mean_dt = sil_dt_sum / sil_count;
                        double mean_dev = sil_dev_sum / sil_count;
                        double mean_read = sil_read_sum / sil_count;

                        char log_line[512];
                        sprintf_s(log_line, sizeof(log_line),
                                  "[%s] | Target: %.1fHz | Detected: %.1fHz | Real: %.1fHz | Err%%: %.3f%% | Jitter(us): mean=%.3f, max=%.3f | dt(us) min/mean/max: %.2f/%.2f/%.2f | Latency(us) min/mean/max: %.2f/%.2f/%.2f | Count: %llu\n",
                                  time_str,
                                  target_hz,
                                  phys_hz,
                                  real_hz,
                                  err_pct,
                                  mean_dev,
                                  sil_dev_max,
                                  sil_dt_min,
                                  mean_dt,
                                  sil_dt_max,
                                  sil_read_min,
                                  mean_read,
                                  sil_read_max,
                                  (unsigned long long)sil_count);
                        log_f << log_line;
                    }
                } catch (...) {}

                // Reset stats
                sil_count = 0;
                sil_dt_sum = 0.0;
                sil_dt_max = 0.0;
                sil_dt_min = 1e18;
                sil_dev_sum = 0.0;
                sil_dev_max = 0.0;
                sil_read_sum = 0.0;
                sil_read_max = 0.0;
                sil_read_min = 1e18;
                
                last_report_sim_time = imu_data.timestamp_ns;
                last_report_wall_time = now_wall_report;
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
        printf("[SIL_App] Log file saved to %s.\n", flight_log_name.c_str());
    }

    // --- Print performance log location summary ---
    {
        std::string perf_log_txt = log_dir + "perf_log_" + std::to_string(hz_tag) + "hz.log";
        printf("[SIL_App] Performance summary log saved to %s.\n", perf_log_txt.c_str());
    }

    return 0;
}
