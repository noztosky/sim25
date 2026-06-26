// jitter_bench.cpp
// Fixed-rate scheduler quality bench for the CURRENT method (coarse sleep + spin).
// NO SHM / NO AirSim. Runs one or more target rates and prints a compact table.
//
// Metrics per rate:
//   detected Hz : achieved ticks / elapsed                                  [검출 Hz]
//   latency(us) : deadline error = actual_fire - scheduled_deadline         [지연]
//                 mean, max
//   jitter(us)  : inter-tick interval deviation = |interval - period|       [지터]
//                 mean, p99, max  (period stability)
//   core%       : process CPU time / wall (100% = one full core)            [코어 점유율]
//                 measured via GetProcessTimes (no external tool needed)
//
// Usage:
//   jitter_bench.exe                    -> 5 s each at 1000 4000 8000
//   jitter_bench.exe 5 1000 4000 8000   -> custom seconds + rates
//   jitter_bench.exe 10 2000            -> 10 s at 2000 Hz
#include <windows.h>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <cmath>
#pragma comment(lib, "winmm.lib")

using clk = std::chrono::steady_clock;

static double process_cpu_seconds() {
    FILETIME c, e, k, u;
    GetProcessTimes(GetCurrentProcess(), &c, &e, &k, &u);
    auto to_s = [](FILETIME f) {
        ULARGE_INTEGER x; x.LowPart = f.dwLowDateTime; x.HighPart = f.dwHighDateTime;
        return (double)x.QuadPart * 1e-7; // 100ns units -> seconds
    };
    return to_s(k) + to_s(u); // kernel + user
}

struct Row {
    double target, detected, errpct;
    double lat_mean, lat_max;
    double jit_mean, jit_p99, jit_max;
    double core;
};

static Row run_one(double target_hz, int secs) {
    using namespace std::chrono;
    const auto period = nanoseconds((long long)(1e9 / target_hz));
    const double period_us = 1e6 / target_hz;

    std::vector<double> lat, jit;
    lat.reserve((size_t)(target_hz * secs) + 16);
    jit.reserve((size_t)(target_hz * secs) + 16);

    timeBeginPeriod(1);
    const double cpu0 = process_cpu_seconds();
    const auto start = clk::now();
    auto next = start + period;
    const auto stop = start + seconds(secs);
    auto prev = start;
    long long ticks = 0;

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
        if (next < w) next = w + period; // realign if we fell behind
        ticks++;
    }
    const auto wall_end = clk::now();
    const double cpu1 = process_cpu_seconds();
    timeEndPeriod(1);

    const double wall = duration<double>(wall_end - start).count();

    Row r{};
    r.target   = target_hz;
    r.detected = ticks / wall;
    r.errpct   = 100.0 * (r.detected - target_hz) / target_hz;

    r.lat_mean = 0; r.lat_max = 0;
    for (double x : lat) { r.lat_mean += x; if (x > r.lat_max) r.lat_max = x; }
    if (!lat.empty()) r.lat_mean /= lat.size();

    std::sort(jit.begin(), jit.end());
    r.jit_mean = 0; for (double x : jit) r.jit_mean += x;
    if (!jit.empty()) r.jit_mean /= jit.size();
    auto pct = [&](double p) { return jit.empty() ? 0.0 : jit[(size_t)(p * (jit.size() - 1))]; };
    r.jit_p99 = pct(0.99);
    r.jit_max = jit.empty() ? 0.0 : jit.back();

    r.core = (wall > 0) ? 100.0 * (cpu1 - cpu0) / wall : 0.0;
    return r;
}

int main(int argc, char** argv) {
    int secs = 5;
    std::vector<double> rates;
    if (argc >= 2) secs = atoi(argv[1]);
    if (secs <= 0) secs = 5;
    for (int i = 2; i < argc; ++i) {
        double hz = atof(argv[i]);
        if (hz > 0) rates.push_back(hz);
    }
    if (rates.empty()) { rates = {1000, 4000, 8000}; }

    SYSTEM_INFO si; GetSystemInfo(&si);
    printf("[jitter_bench] method=coarse-sleep+spin (current) | no SHM/AirSim | %d s/rate | %lu logical cores\n",
           secs, (unsigned long)si.dwNumberOfProcessors);
    printf("\n");
    printf(" target | detected   err%%  | latency us      | jitter us  |dt-T|     | core\n");
    printf("   Hz    |   Hz             | mean      max   | mean    p99      max  |  %%\n");
    printf("--------+------------------+-----------------+-----------------------+------\n");

    std::vector<Row> rows;
    for (double hz : rates) rows.push_back(run_one(hz, secs));

    for (const Row& r : rows) {
        printf(" %6.0f | %8.1f  %+5.2f | %7.2f %8.2f | %6.2f %7.2f %8.2f | %4.0f\n",
               r.target, r.detected, r.errpct,
               r.lat_mean, r.lat_max,
               r.jit_mean, r.jit_p99, r.jit_max,
               r.core);
    }
    printf("--------+------------------+-----------------+-----------------------+------\n");
    printf("\n");
    printf("legend: latency=fire-deadline(지연) | jitter=|interval-period|(지터) | core 100%%=one full core(점유율)\n");
    return 0;
}
