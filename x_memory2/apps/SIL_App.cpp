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

// ---- Tunable flight-control parameters (loadable from a params file at startup) ----
// Lets you tune PID/hover without recompiling: edit the file and rerun SIL_App.
struct TuneParams {
    float hover_throttle;
    // altitude PID
    float alt_p, alt_i, alt_d, alt_ilim, alt_min, alt_max, alt_dfilt;
    // roll/pitch rate PID (inner loop)
    float rate_p, rate_i, rate_d, rate_clamp, rate_ilim, rate_dfilt;
    // yaw rate PID
    float yaw_p, yaw_i, yaw_d, yaw_clamp;
    // outer angle->rate gains
    float again_roll, again_pitch, again_yaw;
};

static TuneParams defaultTuneParams(bool is_z30) {
    TuneParams p{};
    if (is_z30) {
        p.hover_throttle = 0.74f;
        p.alt_p = 0.12f; p.alt_i = 0.03f; p.alt_d = 0.14f; p.alt_ilim = 0.15f; p.alt_min = -0.30f; p.alt_max = 0.10f; p.alt_dfilt = 10.0f;
        p.rate_p = 0.10f; p.rate_i = 0.02f; p.rate_d = 0.030f; p.rate_clamp = 0.20f; p.rate_ilim = 0.10f; p.rate_dfilt = 20.0f;
        p.yaw_p = 0.15f; p.yaw_i = 0.05f; p.yaw_d = 0.0f; p.yaw_clamp = 0.15f;
        p.again_roll = 3.0f; p.again_pitch = 3.0f; p.again_yaw = 1.5f;
    } else {
        p.hover_throttle = 0.59f;
        p.alt_p = 0.18f; p.alt_i = 0.04f; p.alt_d = 0.10f; p.alt_ilim = 0.20f; p.alt_min = -0.25f; p.alt_max = 0.25f; p.alt_dfilt = 10.0f;
        p.rate_p = 0.08f; p.rate_i = 0.02f; p.rate_d = 0.005f; p.rate_clamp = 0.15f; p.rate_ilim = 0.10f; p.rate_dfilt = 20.0f;
        p.yaw_p = 0.15f; p.yaw_i = 0.05f; p.yaw_d = 0.0f; p.yaw_clamp = 0.15f;
        p.again_roll = 4.5f; p.again_pitch = 5.0f; p.again_yaw = 1.5f;
    }
    return p;
}

// Load key=value lines (# comments allowed) from `path`, overriding fields of `p`.
// Returns number of keys applied (-1 if file could not be opened).
static int loadTuneParams(const std::string& path, TuneParams& p) {
    std::ifstream f(path);
    if (!f.is_open()) return -1;
    int applied = 0;
    std::string line;
    while (std::getline(f, line)) {
        // strip comments and whitespace
        size_t hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);
        auto trim = [](std::string& s) {
            size_t a = s.find_first_not_of(" \t\r\n");
            size_t b = s.find_last_not_of(" \t\r\n");
            if (a == std::string::npos) { s.clear(); return; }
            s = s.substr(a, b - a + 1);
        };
        trim(key); trim(val);
        if (key.empty() || val.empty()) continue;
        float v;
        try { v = std::stof(val); } catch (...) { continue; }
        bool ok = true;
        if (key == "hover_throttle") p.hover_throttle = v;
        else if (key == "alt_p") p.alt_p = v;
        else if (key == "alt_i") p.alt_i = v;
        else if (key == "alt_d") p.alt_d = v;
        else if (key == "alt_ilim") p.alt_ilim = v;
        else if (key == "alt_min") p.alt_min = v;
        else if (key == "alt_max") p.alt_max = v;
        else if (key == "alt_dfilt") p.alt_dfilt = v;
        else if (key == "rate_p") p.rate_p = v;
        else if (key == "rate_i") p.rate_i = v;
        else if (key == "rate_d") p.rate_d = v;
        else if (key == "rate_clamp") p.rate_clamp = v;
        else if (key == "rate_ilim") p.rate_ilim = v;
        else if (key == "rate_dfilt") p.rate_dfilt = v;
        else if (key == "yaw_p") p.yaw_p = v;
        else if (key == "yaw_i") p.yaw_i = v;
        else if (key == "yaw_d") p.yaw_d = v;
        else if (key == "yaw_clamp") p.yaw_clamp = v;
        else if (key == "again_roll") p.again_roll = v;
        else if (key == "again_pitch") p.again_pitch = v;
        else if (key == "again_yaw") p.again_yaw = v;
        else ok = false;
        if (ok) ++applied;
    }
    return applied;
}

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

// Runtime control-flow requests set by the UDP listener, consumed by the main loop
std::atomic<bool> g_reload_pid_request(false);  // "pid": re-read params file, re-apply gains live
std::atomic<bool> g_reset_request(false);       // "reset": return to pre-takeoff (reset sim + state)

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

                // Word commands first (no numeric args): "pid" reloads params, "reset" returns to pre-takeoff
                char word[64] = {0};
                if (sscanf(recv_buf, " %63s", word) == 1) {
                    if (_stricmp(word, "pid") == 0) {
                        g_reload_pid_request = true;
                        printf("\n[UDP Server] Received command: PID reload\n");
                        fflush(stdout);
                        continue;
                    }
                    if (_stricmp(word, "reset") == 0) {
                        g_reset_request = true;
                        printf("\n[UDP Server] Received command: RESET to pre-takeoff\n");
                        fflush(stdout);
                        continue;
                    }
                }

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

    double target_hz = 1000.0;
    bool is_takeoff_active = false;
    double target_altitude = 0.0;
    float current_target_alt = 0.0f;
    std::string frame_profile = "default"; // "default" (1kg quad) or "z30" (40kg agri)
    std::string params_path = "";          // optional key=value tuning file; empty = auto by profile

    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "takeoff" && i + 1 < argc) {
            target_altitude = std::stod(argv[i + 1]);
            is_takeoff_active = true;
            i++; // skip next argument
        } else if (arg == "z30" || arg == "Z30" || arg == "eftz30") {
            frame_profile = "z30";
        } else if (arg.rfind("params=", 0) == 0) {
            params_path = arg.substr(7);   // params=<path>
        } else {
            try {
                target_hz = std::stod(arg);
            } catch (...) {
                // Ignore invalid parameters
            }
        }
    }
    const bool is_z30 = (frame_profile == "z30");
    printf("[SIL_App] Frame profile: %s\n", frame_profile.c_str());

    // Build tuning params: start from profile defaults, then override from file if present.
    TuneParams tune = defaultTuneParams(is_z30);
    if (params_path.empty())
        params_path = is_z30 ? "pid_params_z30.txt" : "pid_params.txt";
    int applied = loadTuneParams(params_path, tune);
    if (applied < 0)
        printf("[SIL_App] Tuning: no params file '%s' -> using built-in %s defaults\n", params_path.c_str(), frame_profile.c_str());
    else
        printf("[SIL_App] Tuning: loaded %d keys from '%s' (overriding %s defaults)\n", applied, params_path.c_str(), frame_profile.c_str());
    printf("[SIL_App] hover=%.3f | alt PID=%.3f/%.3f/%.3f lim[%.2f..%.2f] | rate PID=%.3f/%.3f/%.3f clamp=%.2f | angle-gain r/p/y=%.1f/%.1f/%.1f\n",
           tune.hover_throttle, tune.alt_p, tune.alt_i, tune.alt_d, tune.alt_min, tune.alt_max,
           tune.rate_p, tune.rate_i, tune.rate_d, tune.rate_clamp, tune.again_roll, tune.again_pitch, tune.again_yaw);

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

    // Version header: run start + build (compile) timestamp so each log/console
    // self-identifies which binary and settings produced it.
    char run_start_str[32];
    {
        time_t t = time(nullptr);
        struct tm tm_info;
        localtime_s(&tm_info, &t);
        strftime(run_start_str, sizeof(run_start_str), "%Y-%m-%d %H:%M:%S", &tm_info);
    }
    printf("[SIL_App] run_start: %s | build: %s %s | target_hz: %.0f%s\n",
           run_start_str, __DATE__, __TIME__, target_hz, is_takeoff_active ? " | takeoff" : "");

    // Open CSV flight log
    log_file.open(flight_log_name);
    if (log_file.is_open()) {
        log_file << "# run_start: " << run_start_str
                 << " | build: " << __DATE__ << " " << __TIME__
                 << " | target_hz: " << hz_tag
                 << (is_takeoff_active ? " | takeoff" : "") << "\n";
        log_file << "timestamp_ns,dt,raw_baro_alt,gps_alt,ekf_alt,ekf_vel_z,pid_target,pid_out_throttle,"
                    "gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,"
                    "ekf_roll_deg,ekf_pitch_deg,ekf_yaw_deg,"
                    "tgt_roll_deg,tgt_pitch_deg,tgt_yaw_deg,"
                    "pwm_fr,pwm_rl,pwm_fl,pwm_rr\n";
    }

    // Clear performance log (+ version header)
    {
        std::ofstream perf_f(perf_log_name, std::ios::out | std::ios::trunc);
        perf_f << "# run_start: " << run_start_str
               << " | build: " << __DATE__ << " " << __TIME__
               << " | target_hz: " << hz_tag << "\n";
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

    // Setup PID Controllers from tuning params (file-loadable + live-reloadable via "pid" command)
    float att_rate_gain_rp = 0.0f, att_rate_gain_p = 0.0f, att_rate_gain_y = 0.0f;
    float hover_throttle = 0.0f;
    PID pid_alt, pid_roll, pid_pitch, pid_yaw;

    // Applies a TuneParams to all controllers + gains + hover (used at init and on live reload).
    auto applyTune = [&](const TuneParams& t) {
        att_rate_gain_rp = t.again_roll;
        att_rate_gain_p  = t.again_pitch;
        att_rate_gain_y  = t.again_yaw;
        hover_throttle   = t.hover_throttle;
        pid_alt.set_gains(t.alt_p, t.alt_i, t.alt_d, 0.0f);
        pid_alt.set_limits(t.alt_ilim, t.alt_min, t.alt_max);
        pid_alt.set_d_filter_hz(t.alt_dfilt);
        pid_roll.set_gains(t.rate_p, t.rate_i, t.rate_d, 0.0f);
        pid_roll.set_limits(t.rate_ilim, -t.rate_clamp, t.rate_clamp);
        pid_roll.set_d_filter_hz(t.rate_dfilt);
        pid_pitch.set_gains(t.rate_p, t.rate_i, t.rate_d, 0.0f);
        pid_pitch.set_limits(t.rate_ilim, -t.rate_clamp, t.rate_clamp);
        pid_pitch.set_d_filter_hz(t.rate_dfilt);
        pid_yaw.set_gains(t.yaw_p, t.yaw_i, t.yaw_d, 0.0f);
        pid_yaw.set_limits(0.10f, -t.yaw_clamp, t.yaw_clamp);
    };
    applyTune(tune);

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

                // --- Live PID reload ("pid" command): re-read params file, re-apply gains in place ---
                if (g_reload_pid_request.exchange(false)) {
                    TuneParams nt = defaultTuneParams(is_z30);
                    int n = loadTuneParams(params_path, nt);
                    tune = nt;
                    applyTune(tune);
                    printf("[SIL_App] PID reloaded (%d keys from '%s'): hover=%.3f | alt=%.3f/%.3f/%.3f lim[%.2f..%.2f] | rate=%.3f/%.3f/%.3f clamp=%.2f | angle r/p/y=%.1f/%.1f/%.1f\n",
                           (n < 0 ? 0 : n), params_path.c_str(), tune.hover_throttle,
                           tune.alt_p, tune.alt_i, tune.alt_d, tune.alt_min, tune.alt_max,
                           tune.rate_p, tune.rate_i, tune.rate_d, tune.rate_clamp,
                           tune.again_roll, tune.again_pitch, tune.again_yaw);
                    fflush(stdout);
                }

                // --- Reset to pre-takeoff ("reset" command): stop autopilot, zero state, reset sim ---
                if (g_reset_request.exchange(false)) {
                    is_takeoff_active = false;
                    target_altitude = 0.0;
                    current_target_alt = 0.0f;
                    current_target_roll = current_target_pitch = current_target_yaw = 0.0f;
                    cmd_end_time_ns = 0;
                    cmd_active_axis = '\0';
                    yaw_ref_set = false;
                    pid_alt.reset(); pid_roll.reset(); pid_pitch.reset(); pid_yaw.reset();
                    pwm_data.pwm_values = { 0.5f, 0.5f, 0.5f, 0.5f };
                    // Reposition the vehicle in the simulator (back to spawn) via RPC
                    try {
                        RPC_Driver rpc;
                        if (rpc.connect()) {
                            rpc.get_client()->reset();
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));
                            rpc.get_client()->enableApiControl(true);
                            rpc.get_client()->armDisarm(true);
                            rpc.close();
                            printf("[SIL_App] RESET done: vehicle repositioned, autopilot off, PIDs cleared. Send 'a <alt>' to take off again.\n");
                        } else {
                            printf("[SIL_App] RESET (state cleared) but RPC unavailable -> vehicle not repositioned.\n");
                        }
                    } catch (const std::exception& e) {
                        printf("[SIL_App] RESET warning: RPC reset failed: %s\n", e.what());
                    }
                    ekf.init(); // re-init estimator after teleport to avoid a transient
                    fflush(stdout);
                }

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

                    // EKF attitude (deg) for dynamic-maneuver verification
                    float log_r = 0.0f, log_p = 0.0f, log_y = 0.0f;
                    if (has_est) {
                        AttitudeUtils::computeEulerDeg(est.quat[0], est.quat[1], est.quat[2], est.quat[3],
                                                       log_r, log_p, log_y);
                    }
                    const float RAD2DEG = 57.2957795f;

                    log_file << imu_data.timestamp_ns << ","
                             << dt << ","
                             << current_altitude << ","
                             << g_alt << ","
                             << e_alt << ","
                             << e_vel_z << ","
                             << (float)target_altitude << ","
                             << pwm_data.pwm_values[0] << ","
                             << imu_data.gyro[0] << "," << imu_data.gyro[1] << "," << imu_data.gyro[2] << ","
                             << imu_data.accel[0] << "," << imu_data.accel[1] << "," << imu_data.accel[2] << ","
                             << log_r << "," << log_p << "," << log_y << ","
                             << current_target_roll * RAD2DEG << ","
                             << current_target_pitch * RAD2DEG << ","
                             << current_target_yaw * RAD2DEG << ","
                             << pwm_data.pwm_values[0] << "," << pwm_data.pwm_values[1] << ","
                             << pwm_data.pwm_values[2] << "," << pwm_data.pwm_values[3] << "\n";

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

                    float target_roll_rate = att_rate_gain_rp * roll_err;
                    float target_pitch_rate = att_rate_gain_p * pitch_err;
                    float target_yaw_rate = att_rate_gain_y * yaw_err;

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
                if (print_counter >= (int)target_hz) { // ~1 second interval
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
