// jitter_bench_shm.cpp
// Same fixed-rate scheduler as jitter_bench, but ATTACHED TO AIRSIM:
// each tick READS IMU telemetry from AirSim's shared memory (real per-tick work),
// and does NOT send anything back (no PWM submit, no downstream SHM write).
// Goal: check whether reading from AirSim perturbs the latency/jitter we measured
// on the empty-loop bench, while still occupying one core (coarse sleep + spin).
//
// Extra columns vs jitter_bench:
//   read us : imu.read() (SHM consume) latency           mean / max
//   new%    : fraction of ticks that got at least one NEW sample (data freshness)
//
// Requires AirSim running and publishing to the "AirSimXsim" shared memory.
//
// Usage:
//   jitter_bench_shm.exe                    -> 5 s each at 1000 4000 8000
//   jitter_bench_shm.exe 5 8000             -> 5 s at 8000 Hz
//   jitter_bench_shm.exe 10 1000 4000 8000  -> custom
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <cmath>

#include "core/IMU_IF.hpp"
#include "drivers/shm/SHM_Driver.hpp"
#include "modules/imu/IMU_SimSHM.hpp"

#pragma comment(lib, "winmm.lib")

using clk = std::chrono::steady_clock;

static double process_cpu_seconds() {
    FILETIME c, e, k, u;
    GetProcessTimes(GetCurrentProcess(), &c, &e, &k, &u);
    auto to_s = [](FILETIME f) {
        ULARGE_INTEGER x; x.LowPart = f.dwLowDateTime; x.HighPart = f.dwHighDateTime;
        return (double)x.QuadPart * 1e-7;
    };
    return to_s(k) + to_s(u);
}

struct Row {
    double target, detected, errpct;
    double lat_mean, lat_max;
    double jit_mean, jit_p99, jit_max;
    double read_mean, read_max;
    double new_pct;
    double core;
};

static Row run_one(SHM_Driver& driver, double target_hz, int secs) {
    using namespace std::chrono;
    const auto period = nanoseconds((long long)(1e9 / target_hz));
    const double period_us = 1e6 / target_hz;

    IMU_SimSHM imu(driver); // fresh consumer seq for this rate
    imu.init();
    IMUData imu_data{};

    std::vector<double> lat, jit, rd;
    lat.reserve((size_t)(target_hz * secs) + 16);
    jit.reserve((size_t)(target_hz * secs) + 16);
    rd.reserve((size_t)(target_hz * secs) + 16);

    timeBeginPeriod(1);
    const double cpu0 = process_cpu_seconds();
    const auto start = clk::now();
    auto next = start + period;
    const auto stop = start + seconds(secs);
    auto prev = start;
    long long ticks = 0, got = 0;

    while (clk::now() < stop) {
        auto coarse = next - microseconds(1000);
        if (coarse > clk::now())
            std::this_thread::sleep_until(coarse);
        while (clk::now() < next) { YieldProcessor(); }

        auto w = clk::now();
        lat.push_back(duration<double, std::micro>(w - next).count());
        double interval = duration<double, std::micro>(w - prev).count();
        jit.push_back(std::fabs(interval - period_us));
        prev = w;
        next += period;
        if (next < w) next = w + period;

        // --- real per-tick work: read IMU from AirSim SHM (NO send back) ---
        auto r0 = clk::now();
        bool ok = imu.read(imu_data);
        auto r1 = clk::now();
        rd.push_back(duration<double, std::micro>(r1 - r0).count());
        if (ok) got++;

        ticks++;
    }
    const auto wall_end = clk::now();
    const double cpu1 = process_cpu_seconds();
    timeEndPeriod(1);

    const double wall = duration<double>(wall_end - start).count();

    Row r{};
    r.target = target_hz;
    r.detected = ticks / wall;
    r.errpct = 100.0 * (r.detected - target_hz) / target_hz;

    auto mean_max = [](const std::vector<double>& v, double& m, double& mx) {
        m = 0; mx = 0; for (double x : v) { m += x; if (x > mx) mx = x; } if (!v.empty()) m /= v.size();
    };
    mean_max(lat, r.lat_mean, r.lat_max);
    mean_max(rd, r.read_mean, r.read_max);

    std::sort(jit.begin(), jit.end());
    r.jit_mean = 0; for (double x : jit) r.jit_mean += x;
    if (!jit.empty()) r.jit_mean /= jit.size();
    auto pct = [&](double p) { return jit.empty() ? 0.0 : jit[(size_t)(p * (jit.size() - 1))]; };
    r.jit_p99 = pct(0.99);
    r.jit_max = jit.empty() ? 0.0 : jit.back();

    r.new_pct = ticks ? 100.0 * (double)got / (double)ticks : 0.0;
    r.core = (wall > 0) ? 100.0 * (cpu1 - cpu0) / wall : 0.0;
    return r;
}

int main(int argc, char** argv) {
    int secs = 5;
    std::vector<double> rates;
    if (argc >= 2) secs = atoi(argv[1]);
    if (secs <= 0) secs = 5;
    for (int i = 2; i < argc; ++i) { double hz = atof(argv[i]); if (hz > 0) rates.push_back(hz); }
    if (rates.empty()) rates = {1000, 4000, 8000};

    SYSTEM_INFO si; GetSystemInfo(&si);
    printf("[jitter_bench_shm] method=coarse-sleep+spin | READ AirSim SHM, NO send | %d s/rate | %lu cores\n",
           secs, (unsigned long)si.dwNumberOfProcessors);

    SHM_Driver driver("AirSimXsim");
    if (!driver.connect()) {
        printf("\n[ERROR] Could not connect to AirSim SHM 'AirSimXsim'.\n");
        printf("        Start AirSim (publishing telemetry) first, then re-run.\n");
        return 1;
    }
    printf("[ok] connected to AirSim SHM 'AirSimXsim'. sim telem_tx_hz=%.0f (current)\n",
           driver.get_xsim().get_telem_tx_hz());

    printf("\n");
    printf(" target | detected   err%%  | latency us      | jitter us |dt-T|     | read us       | new%% | core\n");
    printf("   Hz    |   Hz             | mean      max   | mean    p99      max  | mean     max  |      |  %%\n");
    printf("--------+------------------+-----------------+-----------------------+---------------+------+------\n");

    std::vector<Row> rows;
    for (double hz : rates) rows.push_back(run_one(driver, hz, secs));

    for (const Row& r : rows) {
        printf(" %6.0f | %8.1f  %+5.2f | %7.2f %8.2f | %6.2f %7.2f %8.2f | %6.2f %7.2f | %4.0f | %4.0f\n",
               r.target, r.detected, r.errpct,
               r.lat_mean, r.lat_max,
               r.jit_mean, r.jit_p99, r.jit_max,
               r.read_mean, r.read_max,
               r.new_pct, r.core);
    }
    printf("--------+------------------+-----------------+-----------------------+---------------+------+------\n");
    printf("\n[sim] telem_tx_hz=%.0f | telem_rx_hz=%.0f | overrun_total=%llu\n",
           driver.get_xsim().get_telem_tx_hz(), driver.get_xsim().get_telem_rx_hz(),
           (unsigned long long)driver.get_xsim().get_telem_overrun_total());
    printf("legend: latency=fire-deadline | jitter=|interval-period| | read=SHM consume | new%%=ticks with fresh sample\n");
    driver.close();
    return 0;
}
