// Compare scheduler WAIT methods at a target rate (no SHM, no work).
//   wait = hr    : high-resolution waitable timer (CREATE_WAITABLE_TIMER_HIGH_RESOLUTION)
//   wait = sleep : std::this_thread::sleep_until (standard OS timer, ~1ms granularity)
//   wait = spin  : busy-wait to the deadline (accurate, 100% CPU)
//
// Usage: wait_test.exe [target_hz] [seconds] [hr|sleep|spin]
#include <windows.h>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#pragma comment(lib, "winmm.lib")

#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

int main(int argc, char** argv)
{
    double target_hz = (argc >= 2) ? atof(argv[1]) : 1000.0;
    int    seconds   = (argc >= 3) ? atoi(argv[2]) : 3;
    const char* mode = (argc >= 4) ? argv[3] : "hr";

    timeBeginPeriod(1);
    using clock = std::chrono::steady_clock;

    HANDLE timer = CreateWaitableTimerExW(NULL, NULL,
                       CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
    const bool hr_ok = (timer != NULL);
    if (std::strcmp(mode, "hr") == 0 && !hr_ok)
        printf("[warn] high-res timer not supported, falling back to sleep_until\n");

    const auto period   = std::chrono::nanoseconds((long long)(1000000000.0 / target_hz));
    const auto start    = clock::now();
    auto next           = start + period;
    auto sec_mark       = start + std::chrono::seconds(1);
    const auto end      = start + std::chrono::seconds(seconds);

    long long ticks = 0, sec_ticks = 0, sec_idx = 0;
    std::vector<double> jit;
    jit.reserve((size_t)(target_hz * seconds) + 16);

    printf("[wait_test] target=%.0f Hz | %d s | wait=%s\n", target_hz, seconds, mode);

    while (clock::now() < end) {
        if (std::strcmp(mode, "hr") == 0 && hr_ok) {
            auto rem = next - clock::now();
            long long rem100 = std::chrono::duration_cast<std::chrono::nanoseconds>(rem).count() / 100; // 100ns units
            if (rem100 > 0) {
                LARGE_INTEGER due; due.QuadPart = -rem100; // negative = relative
                SetWaitableTimer(timer, &due, 0, NULL, NULL, FALSE);
                WaitForSingleObject(timer, INFINITE);
            }
        } else if (std::strcmp(mode, "spin") == 0) {
            while (clock::now() < next) { YieldProcessor(); }
        } else { // sleep
            std::this_thread::sleep_until(next);
        }

        auto w = clock::now();
        jit.push_back(std::chrono::duration<double, std::micro>(w - next).count());
        next += period;
        if (next < w) next = w + period;

        ticks++; sec_ticks++;
        if (w >= sec_mark) {
            printf("  [%lld s] %lld ticks\n", ++sec_idx, sec_ticks);
            sec_ticks = 0;
            sec_mark += std::chrono::seconds(1);
        }
    }
    if (timer) CloseHandle(timer);
    timeEndPeriod(1);

    std::sort(jit.begin(), jit.end());
    double sum = 0; for (double x : jit) sum += x;
    auto pct = [&](double p) { return jit[(size_t)(p * (jit.size() - 1))]; };
    printf("\n[result] %.0f Hz | wait=%s | total=%lld over %d s -> %.1f Hz\n",
           target_hz, mode, ticks, seconds, ticks / (double)seconds);
    printf("[result] deadline jitter us: mean=%.2f p50=%.2f p95=%.2f p99=%.2f max=%.2f\n",
           sum / jit.size(), pct(0.50), pct(0.95), pct(0.99), jit.back());
    return 0;
}
