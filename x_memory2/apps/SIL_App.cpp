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
#include <deque>
#include <array>

#pragma comment(lib, "winmm.lib")
#pragma comment(lib, "ws2_32.lib")

std::ofstream log_file;
std::atomic<bool> should_exit(false);

// --- Realistic sensor transport-delay lines -------------------------------------
// The simulator produces every sensor instantly from ground truth; real GPS/baro/mag
// lag the true state by a transport/solution delay. We buffer each sensor and feed
// the EKF the sample that is LATENCY old, so the estimator (and Loiter) behaves like
// a real vehicle. (IMU/gyro left instant: real IMU lag ~1ms is negligible vs 60ms motor lag.)
static const uint64_t GPS_LATENCY_NS  = 120000000ULL; // 120 ms - GPS position/velocity solution delay (dominant)
static const uint64_t BARO_LATENCY_NS = 15000000ULL;  // 15 ms
static const uint64_t MAG_LATENCY_NS  = 8000000ULL;   // 8 ms
template <typename T>
struct DelayLine {
    std::deque<std::pair<uint64_t, T>> buf;
    void push(uint64_t ts, const T& d) { buf.emplace_back(ts, d); if (buf.size() > 2048) buf.pop_front(); }
    // Release the oldest buffered sample once it is at least `latency` old.
    bool pop_due(uint64_t now_ns, uint64_t latency_ns, T& out) {
        if (!buf.empty() && buf.front().first + latency_ns <= now_ns) {
            out = buf.front().second; buf.pop_front(); return true;
        }
        return false;
    }
    void clear() { buf.clear(); }
};

// --- Ground-truth (physics) feedback probe -------------------------------------
// When enabled with the `gt` arg, a low-rate thread polls AirSim RPC for the TRUE
// body attitude + angular velocity, published here so every flight-log row can
// compare three signals side by side:
//   raw SHM gyro (what the app receives)  vs  EKF estimate  vs  TRUE physics.
// Purpose: tell apart a sensor/SHM bug (true rate large but SHM gyro ~0) from an
// estimator bug (true attitude level but EKF says flipped). Off the control loop.
std::atomic<bool>  g_gt_enabled(false);
std::atomic<bool>  g_gt_valid(false);
std::atomic<float> g_gt_roll(0.0f), g_gt_pitch(0.0f), g_gt_yaw(0.0f);   // deg, from TRUE quaternion
std::atomic<float> g_gt_gx(0.0f), g_gt_gy(0.0f), g_gt_gz(0.0f);         // rad/s, TRUE angular velocity
std::atomic<float> g_gt_posN(0.0f), g_gt_posE(0.0f);                    // m, TRUE horizontal position (NED)

// ---- Tunable flight-control parameters (loadable from a params file at startup) ----
// Lets you tune PID/hover without recompiling: edit the file and rerun SIL_App.
struct TuneParams {
    float hover_throttle;
    // Altitude: ArduPilot-style PSC cascade.
    //   alt_p      = PSC_POSZ_P   (position error [m] -> climb-rate target [m/s])
    //   psc_velz_p = PSC_VELZ_P   (climb-rate error -> throttle delta)
    //   alt_i/d    = PSC_VELZ_I/D (velocity-loop I/D)
    //   pilot_spd_up/dn = PILOT_SPEED_UP/DN climb/descent-rate limits [m/s]
    float alt_p, alt_i, alt_d, alt_ilim, alt_min, alt_max, alt_dfilt;
    float psc_velz_p, pilot_spd_up, pilot_spd_dn;
    // roll/pitch rate PID (inner loop)
    float rate_p, rate_i, rate_d, rate_clamp, rate_ilim, rate_dfilt;
    // gyro low-pass ahead of the rate controller (ArduPilot INS_GYRO_FILTER, default 20 Hz)
    float gyro_filt;
    // Loiter: stick is a VELOCITY command (full stick = loi_maxvel m/s); the tilt the
    // position controller may command is capped at loi_maxtilt (deg). Both file-tunable.
    float loi_maxvel, loi_maxtilt;
    // yaw rate PID
    float yaw_p, yaw_i, yaw_d, yaw_clamp;
    // outer angle->rate gains
    float again_roll, again_pitch, again_yaw;
    // pitch feedforward trim for CG offset compensation
    float pitch_trim;
};

static TuneParams defaultTuneParams(bool is_z30) {
    TuneParams p{};
    if (is_z30) {
        p.hover_throttle = 0.74f;
        p.alt_p = 1.0f; p.alt_i = 0.03f; p.alt_d = 0.0f; p.alt_ilim = 0.15f; p.alt_min = -0.30f; p.alt_max = 0.10f; p.alt_dfilt = 10.0f;
        p.psc_velz_p = 0.15f; p.pilot_spd_up = 2.5f; p.pilot_spd_dn = 1.5f;
        p.rate_p = 0.10f; p.rate_i = 0.02f; p.rate_d = 0.030f; p.rate_clamp = 0.20f; p.rate_ilim = 0.10f; p.rate_dfilt = 20.0f;
        p.gyro_filt = 20.0f;
        p.loi_maxvel = 7.0f; p.loi_maxtilt = 30.0f;
        p.yaw_p = 0.15f; p.yaw_i = 0.05f; p.yaw_d = 0.0f; p.yaw_clamp = 0.15f;
        p.again_roll = 3.0f; p.again_pitch = 3.0f; p.again_yaw = 1.5f;
        p.pitch_trim = 0.076f;
    } else {
        p.hover_throttle = 0.59f;
        p.alt_p = 1.0f; p.alt_i = 0.04f; p.alt_d = 0.0f; p.alt_ilim = 0.20f; p.alt_min = -0.25f; p.alt_max = 0.25f; p.alt_dfilt = 10.0f;
        p.psc_velz_p = 0.18f; p.pilot_spd_up = 2.5f; p.pilot_spd_dn = 1.5f;
        p.rate_p = 0.08f; p.rate_i = 0.02f; p.rate_d = 0.005f; p.rate_clamp = 0.15f; p.rate_ilim = 0.10f; p.rate_dfilt = 20.0f;
        p.gyro_filt = 20.0f;
        p.loi_maxvel = 7.0f; p.loi_maxtilt = 30.0f;
        p.yaw_p = 0.15f; p.yaw_i = 0.05f; p.yaw_d = 0.0f; p.yaw_clamp = 0.15f;
        p.again_roll = 4.5f; p.again_pitch = 5.0f; p.again_yaw = 1.5f;
        p.pitch_trim = 0.0f;
    }
    return p;
}

// Load key=value lines (# comments allowed) from `path`, overriding fields of `p`.
// Returns number of keys applied (-1 if file could not be opened).
// In std::stof, catch all exceptions to continue safely.
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
        else if (key == "gyro_filt") p.gyro_filt = v;
        else if (key == "loi_maxvel") p.loi_maxvel = v;
        else if (key == "loi_maxtilt") p.loi_maxtilt = v;
        else if (key == "psc_velz_p") p.psc_velz_p = v;
        else if (key == "pilot_spd_up") p.pilot_spd_up = v;
        else if (key == "pilot_spd_dn") p.pilot_spd_dn = v;
        else if (key == "yaw_p") p.yaw_p = v;
        else if (key == "yaw_i") p.yaw_i = v;
        else if (key == "yaw_d") p.yaw_d = v;
        else if (key == "yaw_clamp") p.yaw_clamp = v;
        else if (key == "again_roll") p.again_roll = v;
        else if (key == "again_pitch") p.again_pitch = v;
        else if (key == "again_yaw") p.again_yaw = v;
        else if (key == "pitch_trim") p.pitch_trim = v;
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

// Combined multi-axis piloting command "c <roll_deg> <pitch_deg> <yaw_off_deg> <dur>":
// sets roll+pitch+yaw setpoints simultaneously for <dur> seconds. Used by the GUI
// piloting tab so the mouse stick / keyboard can command diagonal motion (the single
// r/p/y commands only hold one axis at a time).
bool g_cmb_new = false;
float g_cmb_roll = 0.0f, g_cmb_pitch = 0.0f, g_cmb_yaw = 0.0f, g_cmb_dur = 0.0f;

// Flight mode: 0 = ALTHOLD (manual attitude, altitude held), 1 = LOITER (GPS
// horizontal position hold + altitude). Set via "mode <althold|loiter>". Default LOITER.
std::atomic<int> g_mode_request(-1);   // -1 none, else new mode requested by UDP

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
                    if (_stricmp(word, "mode") == 0) {
                        char m2[64] = {0};
                        if (sscanf(recv_buf, " %*s %63s", m2) == 1) {
                            if (_stricmp(m2, "loiter") == 0) { g_mode_request = 1; printf("\n[UDP Server] Mode -> LOITER\n"); }
                            else if (_stricmp(m2, "althold") == 0 || _stricmp(m2, "alt") == 0) { g_mode_request = 0; printf("\n[UDP Server] Mode -> ALTHOLD\n"); }
                            fflush(stdout);
                        }
                        continue;
                    }
                }

                // Combined piloting command: "c <roll> <pitch> <yaw_off> <dur>" (all deg/sec)
                char caxis = '\0';
                float cr = 0, cp = 0, cy = 0, cd = 0;
                if (sscanf(recv_buf, " %c %f %f %f %f", &caxis, &cr, &cp, &cy, &cd) == 5 &&
                    (caxis == 'c' || caxis == 'C')) {
                    std::lock_guard<std::mutex> lock(g_cmd_mutex);
                    g_cmb_roll = cr; g_cmb_pitch = cp; g_cmb_yaw = cy; g_cmb_dur = cd;
                    g_cmb_new = true;
                    continue;   // don't also parse as a single-axis command
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

// Low-rate ground-truth poller (own RPC connection, ~50 Hz). Kept entirely off the
// control loop so RPC latency never perturbs timing. Publishes to the g_gt_* atomics.
void gt_poll_thread() {
    RPC_Driver rpc;
    bool connected = false;
    while (!should_exit) {
        if (!connected) {
            connected = rpc.connect();
            if (!connected) { g_gt_valid = false; std::this_thread::sleep_for(std::chrono::milliseconds(500)); continue; }
        }
        try {
            auto k = rpc.get_client()->simGetGroundTruthKinematics();
            float r, p, y;
            AttitudeUtils::computeEulerDeg((float)k.pose.orientation.w(), (float)k.pose.orientation.x(),
                                           (float)k.pose.orientation.y(), (float)k.pose.orientation.z(), r, p, y);
            g_gt_roll = r; g_gt_pitch = p; g_gt_yaw = y;
            g_gt_gx = (float)k.twist.angular.x();
            g_gt_gy = (float)k.twist.angular.y();
            g_gt_gz = (float)k.twist.angular.z();
            g_gt_posN = (float)k.pose.position.x();
            g_gt_posE = (float)k.pose.position.y();
            g_gt_valid = true;
        } catch (const std::exception&) {
            g_gt_valid = false; connected = false; rpc.close();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // ~50 Hz
    }
    rpc.close();
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
        } else if (arg == "gt" || arg == "GT") {
            g_gt_enabled = true;
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

    // Auto-detect profile and hz from settings.json if not explicitly specified via command line arguments
    if (argc <= 1) {
        std::vector<std::string> settings_paths = {
            "../../settings.json",
            "../settings.json",
            "settings.json"
        };
        for (const auto& path : settings_paths) {
            std::ifstream sf(path);
            if (sf.is_open()) {
                std::string content((std::istreambuf_iterator<char>(sf)), std::istreambuf_iterator<char>());
                // Check for "Model": "EFTZ30"
                if (content.find("\"Model\"") != std::string::npos && 
                    (content.find("\"EFTZ30\"") != std::string::npos || content.find("\"eftz30\"") != std::string::npos)) {
                    frame_profile = "z30";
                    printf("[SIL_App] Auto-detected Model: EFTZ30 from %s\n", path.c_str());
                }
                // Check for "PhysicsLoopPeriod"
                size_t pos = content.find("\"PhysicsLoopPeriod\"");
                if (pos != std::string::npos) {
                    size_t colon = content.find(":", pos);
                    if (colon != std::string::npos) {
                        size_t end = content.find_first_of(",}", colon);
                        std::string val_str = content.substr(colon + 1, end - colon - 1);
                        try {
                            int period = std::stoi(val_str);
                            if (period > 0) {
                                target_hz = 1000000000.0 / period;
                                printf("[SIL_App] Auto-detected PhysicsLoopPeriod from %s: %d -> target_hz: %.0f\n", path.c_str(), period, target_hz);
                            }
                        } catch (...) {}
                    }
                }
                break;
            }
        }
    }

    const bool is_z30 = (frame_profile == "z30");
    printf("[SIL_App] Frame profile: %s\n", frame_profile.c_str());

    // Build tuning params: start from profile defaults, then override from file if present.
    TuneParams tune = defaultTuneParams(is_z30);
    if (params_path.empty())
        params_path = is_z30 ? "pid_params_z30.txt" : "pid_params.txt";

    std::string resolved_params_path = params_path;
    {
        std::ifstream check_f(resolved_params_path);
        if (!check_f.is_open()) {
            std::vector<std::string> alt_paths = {
                "build/" + params_path,
                "../build/" + params_path,
                "apps/" + params_path,
                "../" + params_path
            };
            for (const auto& alt_p : alt_paths) {
                std::ifstream alt_f(alt_p);
                if (alt_f.is_open()) {
                    resolved_params_path = alt_p;
                    break;
                }
            }
        }
    }

    int applied = loadTuneParams(resolved_params_path, tune);
    if (applied < 0)
        printf("[SIL_App] Tuning: no params file '%s' -> using built-in %s defaults\n", resolved_params_path.c_str(), frame_profile.c_str());
    else
        printf("[SIL_App] Tuning: loaded %d keys from '%s' (overriding %s defaults)\n", applied, resolved_params_path.c_str(), frame_profile.c_str());
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
    std::thread gt_thread;
    if (g_gt_enabled) {
        gt_thread = std::thread(gt_poll_thread);
        printf("[SIL_App] Ground-truth feedback probe ENABLED (RPC ~50Hz) -> extra gt_* columns in flight log\n");
    }

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
                    "pwm_fr,pwm_rl,pwm_fl,pwm_rr,"
                    "gt_roll_deg,gt_pitch_deg,gt_yaw_deg,gt_gyro_x,gt_gyro_y,gt_gyro_z,gt_pos_n,gt_pos_e\n";
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
    float gyro_lpf_hz = 20.0f;               // ArduPilot INS_GYRO_FILTER equivalent
    float gyro_f[3] = { 0.0f, 0.0f, 0.0f };  // filtered gyro state (rate-controller input)
    float loi_maxvel_mps = 7.0f;             // Loiter full-stick velocity command (m/s)
    float loi_maxtilt_deg = 30.0f;           // Loiter tilt cap the position ctrl may command
    float psc_posz_p_v = 1.0f;               // PSC_POSZ_P: alt error -> climb-rate target
    float pilot_spd_up_v = 2.5f, pilot_spd_dn_v = 1.5f;  // PILOT_SPEED_UP/DN (m/s)
    float pitch_trim = 0.0f;
    PID pid_alt, pid_roll, pid_pitch, pid_yaw;

    // Applies a TuneParams to all controllers + gains + hover (used at init and on live reload).
    auto applyTune = [&](const TuneParams& t) {
        att_rate_gain_rp = t.again_roll;
        att_rate_gain_p  = t.again_pitch;
        att_rate_gain_y  = t.again_yaw;
        hover_throttle   = t.hover_throttle;
        pitch_trim       = t.pitch_trim;
        gyro_lpf_hz      = t.gyro_filt;
        loi_maxvel_mps   = t.loi_maxvel;
        loi_maxtilt_deg  = t.loi_maxtilt;
        // pid_alt is the PSC_VELZ stage: climb-rate error -> throttle delta
        pid_alt.set_gains(t.psc_velz_p, t.alt_i, t.alt_d, 0.0f);
        pid_alt.set_limits(t.alt_ilim, t.alt_min, t.alt_max);
        pid_alt.set_d_filter_hz(t.alt_dfilt);
        psc_posz_p_v   = t.alt_p;
        pilot_spd_up_v = t.pilot_spd_up;
        pilot_spd_dn_v = t.pilot_spd_dn;
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
    // Control loop runs at 400 Hz (ArduPilot SCHED_LOOP_RATE default), decimated from the
    // full IMU/EKF rate (1000 Hz). IMU sampling + EKF propagation stay at the sim rate.
    const uint64_t CONTROL_PERIOD_NS = 2500000;   // 400 Hz control loop
    const float    CONTROL_DT = 1.0f / 400.0f;

    IMUData imu_data;
    PWMData pwm_data;
    pwm_data.pwm_values = { 0.0f, 0.0f, 0.0f, 0.0f }; // Zero thrust when disarmed

    uint64_t last_imu_ts = 0;
    uint64_t last_pwm_send_ts = 0;
    uint64_t last_control_ts = 0;   // 400 Hz control-loop scheduler
    uint64_t last_ekf_gnss_ts = 0;
    uint64_t last_ekf_baro_ts = 0;
    uint64_t last_ekf_imu_ts = 0;   // 100 Hz EKF propagation scheduler (ArduPilot-like)
    uint64_t last_ekf_mag_ts = 0;
    int ignore_sensor_frames = 0;

    // Sensor delay lines (feed EKF the delayed samples so latency is realistic)
    DelayLine<GNSSData> gnss_delay;
    DelayLine<BAROData> baro_delay;
    DelayLine<std::array<double, 3>> mag_delay;

    bool yaw_ref_set = false;
    float yaw_ref_rad = 0.0f;

    // Command targets from UDP server
    uint64_t cmd_end_time_ns = 0;
    char cmd_active_axis = '\0';
    float cmd_active_value = 0.0f;
    float current_target_roll = 0.0f;
    float current_target_pitch = 0.0f;
    float current_target_yaw = 0.0f;

    // Combined piloting command state (roll/pitch/yaw held together, radians)
    uint64_t cmb_end_time_ns = 0;
    float cmb_roll_rad = 0.0f, cmb_pitch_rad = 0.0f, cmb_yaw_rad = 0.0f;

    // Flight mode + Loiter (horizontal position hold) state
    int flight_mode = 1;              // default LOITER
    bool loi_valid = false;           // has a loiter target been captured?
    float loi_tgt_N = 0.0f, loi_tgt_E = 0.0f;

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

        if (ignore_sensor_frames > 0) {
            ignore_sensor_frames--;
            pwm_data.pwm_values = { 0.0f, 0.0f, 0.0f, 0.0f };
            act.write(pwm_data);
            continue;
        }

        // Guard against non-finite values (NaN, Inf) in IMU telemetry (e.g. during reset transition)
        if (!std::isfinite(imu_data.accel[0]) || !std::isfinite(imu_data.accel[1]) || !std::isfinite(imu_data.accel[2]) ||
            !std::isfinite(imu_data.gyro[0]) || !std::isfinite(imu_data.gyro[1]) || !std::isfinite(imu_data.gyro[2]) ||
            !std::isfinite(imu_data.mag[0]) || !std::isfinite(imu_data.mag[1]) || !std::isfinite(imu_data.mag[2])) {
            continue;
        }

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

                // Gyro low-pass at the full 1000 Hz sample rate (ArduPilot INS_GYRO_FILTER,
                // default 20 Hz). The rate controller consumes this filtered gyro; the EKF
                // keeps the raw samples.
                {
                    float rc = 1.0f / (2.0f * 3.14159265f * (gyro_lpf_hz > 1.0f ? gyro_lpf_hz : 20.0f));
                    float a = dt / (rc + dt);
                    for (int gi = 0; gi < 3; ++gi)
                        gyro_f[gi] += a * ((float)imu_data.gyro[gi] - gyro_f[gi]);
                }

                // Propagate EKF state using IMU at 100 Hz (ArduPilot EKF3 update rate).
                // Gyro/accel are still SAMPLED at 1000 Hz; the estimator runs at 100 Hz.
                if (imu_data.timestamp_ns - last_ekf_imu_ts >= 10000000ULL) { // 100 Hz
                    ekf.update_imu(imu_data);
                    last_ekf_imu_ts = imu_data.timestamp_ns;
                }

                // --- Read GNSS and Baro (Common Sensor API Layer) ---
                GNSSData gnss_data;
                BAROData baro_data;
                bool has_gnss = gnss.read(gnss_data);
                bool has_baro = baro.read(baro_data);

                // Check for non-finite values (NaN, Inf) in GNSS
                if (has_gnss) {
                    if (!std::isfinite(gnss_data.pos_ned[0]) || !std::isfinite(gnss_data.pos_ned[1]) || !std::isfinite(gnss_data.pos_ned[2]) ||
                        !std::isfinite(gnss_data.vel_ned[0]) || !std::isfinite(gnss_data.vel_ned[1]) || !std::isfinite(gnss_data.vel_ned[2]) ||
                        !std::isfinite(gnss_data.lat) || !std::isfinite(gnss_data.lon) || !std::isfinite(gnss_data.alt)) {
                        has_gnss = false;
                    }
                }

                // Check for non-finite values (NaN, Inf) in Baro
                if (has_baro) {
                    if (!std::isfinite(baro_data.altitude) || !std::isfinite(baro_data.pressure) || !std::isfinite(baro_data.temperature)) {
                        has_baro = false;
                    }
                }

                // Fuse GNSS/Baro/Mag in EKF through realistic transport-delay lines:
                // buffer the fresh sample, then feed the EKF whatever sample is now LATENCY old.
                uint64_t now_ns = imu_data.timestamp_ns;
                // ArduPilot-default sensor rates (sim otherwise delivers them at ~1 kHz):
                //   GPS 5 Hz (GPS_RATE_MS=200), baro 10 Hz, mag 10 Hz.
                if (has_gnss && now_ns - last_ekf_gnss_ts >= 200000000ULL) { // 5 Hz GPS
                    gnss_delay.push(now_ns, gnss_data);
                    last_ekf_gnss_ts = now_ns;
                }
                if (has_baro && now_ns - last_ekf_baro_ts >= 100000000ULL) {  // 10 Hz baro
                    baro_delay.push(now_ns, baro_data);
                    last_ekf_baro_ts = now_ns;
                }
                if ((imu_data.mag[0] != 0.0 || imu_data.mag[1] != 0.0 || imu_data.mag[2] != 0.0) &&
                    (now_ns - last_ekf_mag_ts >= 100000000ULL)) { // 10 Hz mag
                    mag_delay.push(now_ns, {imu_data.mag[0], imu_data.mag[1], imu_data.mag[2]});
                    last_ekf_mag_ts = now_ns;
                }

                GNSSData d_gnss;
                if (gnss_delay.pop_due(now_ns, GPS_LATENCY_NS, d_gnss)) ekf.update_gnss(d_gnss);
                BAROData d_baro;
                if (baro_delay.pop_due(now_ns, BARO_LATENCY_NS, d_baro)) ekf.update_baro(d_baro);
                std::array<double, 3> d_mag;
                if (mag_delay.pop_due(now_ns, MAG_LATENCY_NS, d_mag)) ekf.update_mag(d_mag.data());

                EstimatedState est;
                bool has_est = ekf.get_estimated_state(est);

                // --- Live PID reload ("pid" command): re-read params file, re-apply gains in place ---
                if (g_reload_pid_request.exchange(false)) {
                    TuneParams nt = defaultTuneParams(is_z30);
                    int n = loadTuneParams(resolved_params_path, nt);
                    tune = nt;
                    applyTune(tune);
                    printf("[SIL_App] PID reloaded (%d keys from '%s'): hover=%.3f | alt=%.3f/%.3f/%.3f lim[%.2f..%.2f] | rate=%.3f/%.3f/%.3f clamp=%.2f | angle r/p/y=%.1f/%.1f/%.1f\n",
                           (n < 0 ? 0 : n), resolved_params_path.c_str(), tune.hover_throttle,
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
                    cmb_end_time_ns = 0;
                    loi_valid = false;
                    yaw_ref_set = false;
                    pid_alt.reset(); pid_roll.reset(); pid_pitch.reset(); pid_yaw.reset();
                    gyro_f[0] = gyro_f[1] = gyro_f[2] = 0.0f;
                    pwm_data.pwm_values = { 0.0f, 0.0f, 0.0f, 0.0f };
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
                    ignore_sensor_frames = 200;
                    last_ekf_gnss_ts = 0;
                    last_ekf_baro_ts = 0;
                    last_ekf_mag_ts = 0;
                    gnss_delay.clear();
                    baro_delay.clear();
                    mag_delay.clear();
                    fflush(stdout);
                }

                // Flight-mode switch request
                {
                    int mreq = g_mode_request.exchange(-1);
                    if (mreq == 0 || mreq == 1) {
                        flight_mode = mreq;
                        loi_valid = false;   // recapture loiter target on (re)entry
                        printf("[Autopilot] Flight mode: %s\n", flight_mode == 1 ? "LOITER" : "ALTHOLD");
                        fflush(stdout);
                    }
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

                // Consume combined piloting command (roll/pitch/yaw held simultaneously)
                bool got_cmb = false;
                float cmb_r = 0, cmb_p = 0, cmb_y = 0, cmb_d = 0;
                {
                    std::lock_guard<std::mutex> lock(g_cmd_mutex);
                    if (g_cmb_new) {
                        cmb_r = g_cmb_roll; cmb_p = g_cmb_pitch; cmb_y = g_cmb_yaw; cmb_d = g_cmb_dur;
                        g_cmb_new = false; got_cmb = true;
                    }
                }
                if (got_cmb) {
                    cmb_roll_rad = cmb_r * 0.01745329f;
                    cmb_pitch_rad = cmb_p * 0.01745329f;
                    cmb_yaw_rad = cmb_y * 0.01745329f;
                    cmb_end_time_ns = imu_data.timestamp_ns + static_cast<uint64_t>(cmb_d * 1e9);
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
                             << pwm_data.pwm_values[2] << "," << pwm_data.pwm_values[3] << ","
                             << g_gt_roll.load() << "," << g_gt_pitch.load() << "," << g_gt_yaw.load() << ","
                             << g_gt_gx.load() << "," << g_gt_gy.load() << "," << g_gt_gz.load() << ","
                             << g_gt_posN.load() << "," << g_gt_posE.load() << "\n";

                    static int log_counter = 0;
                    if (++log_counter >= 1000) {
                        log_file.flush();
                        log_counter = 0;
                    }
                }

                // Autopilot control logic — 400 Hz control loop, decimated from the 1000 Hz
                // IMU/EKF rate (ArduPilot structure: fast gyro sampling, slower control loop).
                if (last_control_ts == 0 ||
                    imu_data.timestamp_ns - last_control_ts >= 2 * CONTROL_PERIOD_NS)
                    last_control_ts = imu_data.timestamp_ns - CONTROL_PERIOD_NS;   // (re)sync
                bool control_tick = (imu_data.timestamp_ns - last_control_ts >= CONTROL_PERIOD_NS);
                if (is_takeoff_active && control_tick) {
                    last_control_ts += CONTROL_PERIOD_NS;
                    float dt = CONTROL_DT;   // control-rate dt (1/400), shadows the 1/1000 loop dt
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

                    // Auto-reset if flipped over (Roll or Pitch magnitude > 75 degrees)
                    if (std::abs(roll_deg) > 75.0f || std::abs(pitch_deg) > 75.0f) {
                        printf("\n[Autopilot] EMERGENCY: Drone flipped over (Roll=%.1f, Pitch=%.1f). Triggering auto-reset...\n", roll_deg, pitch_deg);
                        g_reset_request = true;
                    }

                    float roll_rad = roll_deg * 0.01745329f;
                    float pitch_rad = pitch_deg * 0.01745329f;
                    float yaw_rad = yaw_deg * 0.01745329f;

                    if (!yaw_ref_set) {
                        yaw_ref_rad = yaw_rad;
                        yaw_ref_set = true;
                    }

                    // --- Pilot command extraction (roll/pitch/yaw + altitude) ---
                    float pilot_roll = 0.0f, pilot_pitch = 0.0f;
                    float pilot_yaw = yaw_ref_rad;
                    bool pilot_moving = false;
                    current_target_alt = (float)target_altitude;

                    if (imu_data.timestamp_ns < cmd_end_time_ns) {
                        if (cmd_active_axis == 'r') { pilot_roll = cmd_active_value; pilot_moving = true; }
                        else if (cmd_active_axis == 'p') { pilot_pitch = cmd_active_value; pilot_moving = true; }
                        else if (cmd_active_axis == 'y') { pilot_yaw = yaw_ref_rad + cmd_active_value; }
                        else if (cmd_active_axis == 'a') { current_target_alt = cmd_active_value; }
                    }
                    if (imu_data.timestamp_ns < cmb_end_time_ns) {
                        pilot_roll = cmb_roll_rad; pilot_pitch = cmb_pitch_rad;
                        pilot_yaw = yaw_ref_rad + cmb_yaw_rad;
                        if (std::abs(cmb_roll_rad) > 0.017f || std::abs(cmb_pitch_rad) > 0.017f)
                            pilot_moving = true;
                    }

                    current_target_yaw = pilot_yaw;

                    if (flight_mode == 1 && has_est) {
                        // ===== LOITER: GPS horizontal position/velocity hold =====
                        const float G = 9.81f;
                        // Velocity-first semantics: the stick is a VELOCITY command. Full stick
                        // (a tilt command equal to loi_maxtilt from the GUI) maps to loi_maxvel m/s;
                        // the tilt angle is only an actuation CAP, not the command variable.
                        const float LOI_MAXVEL = loi_maxvel_mps;                       // param (default 7 m/s)
                        const float LOI_POS_P = 0.25f;       // position error -> desired velocity (gentle)
                        const float LOI_VEL_P = 1.0f;        // velocity error -> accel (gentle: outer loop
                                                             // must be far slower than the attitude loop)
                        const float LOI_MAXTILT = loi_maxtilt_deg * 0.01745329f;       // param (default 30 deg)
                        const float PILOT_TILT_REF = LOI_MAXTILT;                      // full stick reference

                        float posN = (float)est.pos_ned[0], posE = (float)est.pos_ned[1];
                        float velN = (float)est.vel_ned[0], velE = (float)est.vel_ned[1];
                        float yc = cosf(yaw_rad), ys = sinf(yaw_rad);

                        if (!loi_valid) { loi_tgt_N = posN; loi_tgt_E = posE; loi_valid = true; }

                        float vN_des, vE_des;
                        if (pilot_moving) {
                            // stick fraction (clamped +/-1) -> body-frame velocity request,
                            // so a held key/full stick commands exactly loi_maxvel m/s
                            float ffwd = -pilot_pitch / PILOT_TILT_REF;  // nose-down(-pitch)=fwd
                            float frgt =  pilot_roll  / PILOT_TILT_REF;  // right-roll(+)=right
                            if (ffwd > 1.0f) ffwd = 1.0f; else if (ffwd < -1.0f) ffwd = -1.0f;
                            if (frgt > 1.0f) frgt = 1.0f; else if (frgt < -1.0f) frgt = -1.0f;
                            float vfwd = ffwd * LOI_MAXVEL;
                            float vrgt = frgt * LOI_MAXVEL;
                            vN_des = vfwd * yc - vrgt * ys;
                            vE_des = vfwd * ys + vrgt * yc;
                            loi_tgt_N = posN; loi_tgt_E = posE;   // hold where the stick releases
                        } else {
                            float eN = loi_tgt_N - posN, eE = loi_tgt_E - posE;
                            vN_des = LOI_POS_P * eN; vE_des = LOI_POS_P * eE;
                            float vmag = sqrtf(vN_des * vN_des + vE_des * vE_des);
                            if (vmag > LOI_MAXVEL) { vN_des *= LOI_MAXVEL / vmag; vE_des *= LOI_MAXVEL / vmag; }
                        }
                        float aN = LOI_VEL_P * (vN_des - velN);
                        float aE = LOI_VEL_P * (vE_des - velE);
                        float a_fwd = aN * yc + aE * ys;
                        float a_rgt = -aN * ys + aE * yc;
                        float lp = -a_fwd / G;   // forward accel -> nose down (-pitch)
                        float lr = a_rgt / G;    // rightward accel -> roll right (+)
                        if (lp > LOI_MAXTILT) lp = LOI_MAXTILT; else if (lp < -LOI_MAXTILT) lp = -LOI_MAXTILT;
                        if (lr > LOI_MAXTILT) lr = LOI_MAXTILT; else if (lr < -LOI_MAXTILT) lr = -LOI_MAXTILT;
                        current_target_roll = lr;
                        current_target_pitch = lp;
                    } else {
                        // ===== ALTHOLD: direct manual attitude =====
                        current_target_roll = pilot_roll;
                        current_target_pitch = pilot_pitch;
                        loi_valid = false;   // recapture loiter target when re-entering LOITER
                    }

                    // --- 1. Altitude Control: ArduPilot PSC cascade ---
                    // position -> climb-rate target (PSC_POSZ_P), clamped by PILOT_SPEED_UP/DN,
                    // then climb-rate PID (PSC_VELZ) -> throttle delta. The rate limit is what
                    // structurally prevents altitude overshoot (unlike the old single alt PID).
                    float feedback_alt = has_est ? -est.pos_ned[2] : current_altitude;
                    float climb_rate = has_est ? -(float)est.vel_ned[2] : 0.0f;
                    // Landed I-term relax (ArduPilot-style): while still on/near the ground the
                    // climb-rate target is unmet, which would wind up the integrator and cause a
                    // climb-rate overshoot right after liftoff. Keep the PID cleared until airborne.
                    if (feedback_alt < 0.2f) pid_alt.reset();
                    float climb_tgt = psc_posz_p_v * (current_target_alt - feedback_alt);
                    if (climb_tgt > pilot_spd_up_v) climb_tgt = pilot_spd_up_v;
                    else if (climb_tgt < -pilot_spd_dn_v) climb_tgt = -pilot_spd_dn_v;
                    float alt_output = pid_alt.compute(climb_tgt, climb_rate, dt);
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
                    float cr = pid_roll.compute(target_roll_rate, gyro_f[0], dt);
                    float cp = pid_pitch.compute(target_pitch_rate, gyro_f[1], dt);
                    float cy = pid_yaw.compute(target_yaw_rate, gyro_f[2], dt);

                    // --- 3. Actuator Mixer (airmode-style desaturation) ---
                    // Attitude (roll/pitch/yaw) torque mix per motor, WITHOUT collective.
                    // Order matches pwm_values below: [0]=FR [1]=RL [2]=FL [3]=RR.
                    float att[4];
                    att[0] =  (cp - pitch_trim) - cr + cy; // FR
                    att[1] = -(cp - pitch_trim) + cr + cy; // RL
                    att[2] =  (cp - pitch_trim) + cr - cy; // FL
                    att[3] = -(cp - pitch_trim) - cr - cy; // RR

                    // If the attitude span alone exceeds the full [0,1] throttle range,
                    // scale it down uniformly (last resort: lose magnitude, keep ratios).
                    float amin = att[0], amax = att[0];
                    for (int i = 1; i < 4; ++i) { amin = std::min(amin, att[i]); amax = std::max(amax, att[i]); }
                    float span = amax - amin;
                    if (span > 1.0f) {
                        float s = 1.0f / span;
                        for (int i = 0; i < 4; ++i) att[i] *= s;
                        amin *= s; amax *= s;
                    }

                    // Shift the collective so all four motors fit in [0,1] while preserving
                    // the torque differential (prioritize attitude control over exact altitude).
                    // span<=1 guarantees lo<=hi, so this never clips the differential.
                    float lo = -amin, hi = 1.0f - amax;
                    float coll = throttle < lo ? lo : (throttle > hi ? hi : throttle);

                    auto clamp = [](float v) {
                        return (v < 0.0f) ? 0.0f : ((v > 1.0f) ? 1.0f : v);
                    };

                    pwm_data.pwm_values = {
                        clamp(coll + att[0]), // FR [0]
                        clamp(coll + att[1]), // RL [1]
                        clamp(coll + att[2]), // FL [2]
                        clamp(coll + att[3])  // RR [3]
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
    if (gt_thread.joinable()) {
        gt_thread.join();
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
