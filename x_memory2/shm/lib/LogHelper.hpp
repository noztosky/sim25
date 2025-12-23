#pragma once

#include <string>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>
#include <cstdio>
#ifdef _WIN32
#include <direct.h>
#endif

struct XSimTelemetry; // fwd
struct XSimPwm;       // fwd
class x_xsim;         // fwd

class LogHelper {
public:
    // Create LOGS/timestamp.LOG and return full path (empty if failed)
    std::string openNew(const std::string& root = "LOGS")
    {
        // create directory (best-effort, minimal deps for UE build)
        auto mkdirs = [](const std::string& path){
            #ifdef _WIN32
            size_t pos = 0; 
            while ((pos = path.find_first_of("/\\", pos)) != std::string::npos) {
                std::string dir = path.substr(0, pos);
                if (!dir.empty()) _mkdir(dir.c_str());
                ++pos;
            }
            _mkdir(path.c_str());
            #endif
        };
        mkdirs(root);
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm lt{}; localtime_s(&lt, &t);
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
            lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec);
        std::string path = root + "\\" + std::string(buf) + ".LOG";
        file_.open(path, std::ios::out | std::ios::trunc);
        if (file_.is_open()) {
            file_ << "xsim_client log start: " << path << std::endl;
            file_.flush();
            return path;
        }
        return std::string();
    }

	void logText(const std::string& line)
	{
		std::cout << line << std::endl;
        if (file_.is_open()) { file_ << line << std::endl; file_.flush(); }
	}
	void logControl(
		float roll_deg, float pitch_deg, float yaw_deg,
		float ex, float ey, int d_roll, int d_pitch,
		double i_roll_us, double i_pitch_us,
		int m1, int m2, int m3, int m4,
		int base, const int bias[4], float kp)
	{
		std::ostringstream os;
		os.setf(std::ios::fixed); os << std::setprecision(2);
		os << "ctl: att[r p y]= " << roll_deg << " " << pitch_deg << " " << yaw_deg
		   << ", err[ex ey]= " << ex << " " << ey
		   << ", d[R P]= " << d_roll << " " << d_pitch
		   << ", I[R P]= " << i_roll_us << " " << i_pitch_us
		   << ", mix[FR RL FL RR]= " << m1 << " " << m2 << " " << m3 << " " << m4
		   << ", base= " << base
		   << ", bias= " << bias[0] << " " << bias[1] << " " << bias[2] << " " << bias[3]
		   << ", kp= " << kp;
        logText(os.str());
	}
	void logStats(
		const XSimTelemetry& last_telem, bool have_telem,
		const XSimPwm& last_pwm, bool have_pwm,
		double dt,
		std::atomic<uint64_t>& imu_rx_cnt,
		std::atomic<uint64_t>& mag_rx_cnt,
		std::atomic<uint64_t>& baro_rx_cnt,
		std::atomic<uint64_t>& loc_rx_cnt,
		const x_xsim& xs)
	{
		uint64_t imu_now = imu_rx_cnt.exchange(0);
		uint64_t mag_now = mag_rx_cnt.exchange(0);
		uint64_t baro_now = baro_rx_cnt.exchange(0);
		uint64_t loc_now = loc_rx_cnt.exchange(0);
		int imu_hz = dt > 0.0 ? static_cast<int>(std::lround(imu_now / dt)) : 0;
		int mag_hz = dt > 0.0 ? static_cast<int>(std::lround(mag_now / dt)) : 0;
		int baro_hz = dt > 0.0 ? static_cast<int>(std::lround(baro_now / dt)) : 0;
		int loc_hz = dt > 0.0 ? static_cast<int>(std::lround(loc_now / dt)) : 0;
		int pwm_hz = static_cast<int>(std::lround(xs.get_pwm_tx_hz()));

		std::ostringstream os; os.setf(std::ios::fixed);
		os << std::setprecision(2)
		   << "imu: " << (have_telem? last_telem.gyro[0]*57.29578:0.0) << " " << (have_telem? last_telem.gyro[1]*57.29578:0.0) << " " << (have_telem? last_telem.gyro[2]*57.29578:0.0)
		   << " " << (have_telem? last_telem.acc[0]:0.0) << " " << (have_telem? last_telem.acc[1]:0.0) << " " << (have_telem? last_telem.acc[2]:0.0) << "(" << imu_hz << "hz), "
		   << "mag: " << (have_telem? last_telem.mag[0]:0.0) << " " << (have_telem? last_telem.mag[1]:0.0) << " " << (have_telem? last_telem.mag[2]:0.0) << " (" << mag_hz << "hz) "
		   << "baro: " << (have_telem? last_telem.alt:0.0) << " (" << baro_hz << "hz) "
		   << "loc: " << (have_telem? last_telem.loc_ned[0]:0.0) << " " << (have_telem? last_telem.loc_ned[1]:0.0) << " " << (have_telem? last_telem.loc_ned[2]:0.0) << " (" << loc_hz << "hz) "
		   << std::setprecision(0)
		   << "pwm: " << (have_pwm? last_pwm.rotor1:0) << " " << (have_pwm? last_pwm.rotor2:0) << " " << (have_pwm? last_pwm.rotor3:0) << " " << (have_pwm? last_pwm.rotor4:0) << "  (" << pwm_hz << "hz)";
        logText(os.str());
	}
private:
    std::ofstream file_{};
};


