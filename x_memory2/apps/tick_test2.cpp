// Scheduler test with simulated per-iteration work of random 1~10us.
// mode "sleep": std::this_thread::sleep_for(1-10us)  (shows Windows granularity issue)
// mode "spin" : busy-wait 1-10us                     (accurate work simulation)
#include <windows.h>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#pragma comment(lib, "winmm.lib")

int main(int argc, char** argv) {
    double target_hz = (argc >= 2) ? atof(argv[1]) : 1000.0;
    int seconds = (argc >= 3) ? atoi(argv[2]) : 5;
    const char* mode = (argc >= 4) ? argv[3] : "sleep";
    bool use_spin = (std::strcmp(mode, "spin") == 0);

    timeBeginPeriod(1);
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::nanoseconds((long long)(1000000000.0 / target_hz));
    const auto start = clock::now();
    auto next = start + period;
    auto sec_mark = start + std::chrono::seconds(1);
    const auto end = start + std::chrono::seconds(seconds);

    long long ticks = 0, sec_ticks = 0, sec_idx = 0;
    std::vector<double> jit;
    jit.reserve((size_t)(target_hz * seconds) + 16);

    printf("[test] target=%.0f Hz, %d s, work=%s (random 1-10us per tick)\n", target_hz, seconds, mode);

    while (clock::now() < end) {
        auto coarse = next - std::chrono::microseconds(1000);
        if (coarse > clock::now())
            std::this_thread::sleep_until(coarse);
        while (clock::now() < next) { YieldProcessor(); }

        auto w = clock::now();
        jit.push_back(std::chrono::duration<double, std::micro>(w - next).count());
        next += period;
        if (next < w) next = w + period;

        // --- simulate 1~10 us of per-iteration work ---
        int us = 1 + (rand() % 10);
        if (use_spin) {
            auto t0 = clock::now();
            auto dur = std::chrono::microseconds(us);
            while (clock::now() - t0 < dur) { YieldProcessor(); }
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(us));
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
    printf("\n[result] work=%s: total=%lld over %d s -> %.1f Hz | jitter us mean=%.2f p50=%.2f p95=%.2f p99=%.2f max=%.2f\n",
           mode, ticks, seconds, ticks / (double)seconds,
           sum / jit.size(), pct(0.50), pct(0.95), pct(0.99), jit.back());
    return 0;
}
