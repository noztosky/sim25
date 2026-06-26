// Pure client speed test (no SHM / no AirSim).
// Measures whether the fixed-rate scheduler (sleep_until + spin) holds the
// target rate when each iteration does a simulated workload.
//
// Usage:
//   client_speed.exe [target_hz] [seconds] [mode] [work_min_us] [work_max_us]
//     target_hz    target loop rate (Hz)        default 1000
//     seconds      test duration (s)            default 5
//     mode         work type: spin | sleep | none   default spin
//                    spin  = busy-wait the work (realistic CPU work)
//                    sleep = std::this_thread::sleep_for (Windows ~1ms granularity!)
//                    none  = no per-iteration work (scheduler only)
//     work_min_us  min simulated work per tick  default 1
//     work_max_us  max simulated work per tick  default 10
//
// Examples:
//   client_speed.exe                       -> 1000 Hz, 5 s, spin, 1-10 us
//   client_speed.exe 4000 10 spin 2 8      -> 4 kHz, 10 s, busy-wait 2-8 us
//   client_speed.exe 1000 5 sleep          -> shows Windows sleep granularity issue
//   client_speed.exe 8000 5 none           -> pure scheduler at 8 kHz
#include <windows.h>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#pragma comment(lib, "winmm.lib")

int main(int argc, char** argv)
{
    if (argc >= 2 && (std::strcmp(argv[1], "-h") == 0 || std::strcmp(argv[1], "--help") == 0)) {
        printf("Usage: client_speed.exe [target_hz] [seconds] [mode: spin|sleep|none] [work_min_us] [work_max_us]\n");
        return 0;
    }

    double target_hz   = (argc >= 2) ? atof(argv[1]) : 1000.0;
    int    seconds     = (argc >= 3) ? atoi(argv[2]) : 5;
    const char* mode   = (argc >= 4) ? argv[3] : "spin";
    int    work_min_us = (argc >= 5) ? atoi(argv[4]) : 1;
    int    work_max_us = (argc >= 6) ? atoi(argv[5]) : 10;

    if (target_hz <= 0) target_hz = 1000.0;
    if (seconds   <= 0) seconds = 5;
    if (work_min_us < 0) work_min_us = 0;
    if (work_max_us < work_min_us) work_max_us = work_min_us;

    const bool use_spin  = (std::strcmp(mode, "spin")  == 0);
    const bool use_sleep = (std::strcmp(mode, "sleep") == 0);
    const int  work_span = work_max_us - work_min_us + 1;

    timeBeginPeriod(1);
    using clock = std::chrono::steady_clock;
    const auto period   = std::chrono::nanoseconds((long long)(1000000000.0 / target_hz));
    const auto start    = clock::now();
    auto next           = start + period;
    auto sec_mark       = start + std::chrono::seconds(1);
    const auto end      = start + std::chrono::seconds(seconds);

    long long ticks = 0, sec_ticks = 0, sec_idx = 0;
    std::vector<double> jit;
    jit.reserve((size_t)(target_hz * seconds) + 16);

    printf("[client_speed] target=%.0f Hz | %d s | work=%s %d-%d us\n",
           target_hz, seconds, mode, work_min_us, work_max_us);

    while (clock::now() < end) {
        // --- fixed-rate scheduler: coarse sleep then spin to the deadline ---
        auto coarse = next - std::chrono::microseconds(1000);
        if (coarse > clock::now())
            std::this_thread::sleep_until(coarse);
        while (clock::now() < next) { YieldProcessor(); }

        auto w = clock::now();
        jit.push_back(std::chrono::duration<double, std::micro>(w - next).count());
        next += period;
        if (next < w) next = w + period;

        // --- simulated per-iteration workload ---
        int us = work_min_us + (work_span > 0 ? (rand() % work_span) : 0);
        if (us > 0) {
            if (use_spin) {
                auto t0 = clock::now();
                auto dur = std::chrono::microseconds(us);
                while (clock::now() - t0 < dur) { YieldProcessor(); }
            } else if (use_sleep) {
                std::this_thread::sleep_for(std::chrono::microseconds(us));
            }
            // mode "none": do nothing
        }

        ticks++; sec_ticks++;
        if (w >= sec_mark) {
            printf("  [%lld s] %lld ticks\n", ++sec_idx, sec_ticks);
            sec_ticks = 0;
            sec_mark += std::chrono::seconds(1);
        }
    }
    timeEndPeriod(1);

    std::sort(jit.begin(), jit.end());
    double sum = 0; for (double x : jit) sum += x;
    auto pct = [&](double p) { return jit[(size_t)(p * (jit.size() - 1))]; };
    printf("\n[result] %.0f Hz target | work=%s %d-%d us | total=%lld over %d s -> %.1f Hz\n",
           target_hz, mode, work_min_us, work_max_us, ticks, seconds, ticks / (double)seconds);
    printf("[result] deadline jitter us: mean=%.2f p50=%.2f p95=%.2f p99=%.2f max=%.2f\n",
           sum / jit.size(), pct(0.50), pct(0.95), pct(0.99), jit.back());
    return 0;
}
