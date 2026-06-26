// Pure scheduler test: NO SHM, NO AirSim — just count how many times the
// sleep_until + spin loop fires per second at a target rate, and report jitter.
#include <windows.h>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <algorithm>
#pragma comment(lib, "winmm.lib")

int main(int argc, char** argv) {
    double target_hz = (argc >= 2) ? atof(argv[1]) : 1000.0;
    int seconds = (argc >= 3) ? atoi(argv[2]) : 5;

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

    printf("[tick_test] target=%.0f Hz, %d s (no comms, scheduler only)\n", target_hz, seconds);

    while (clock::now() < end) {
        auto coarse = next - std::chrono::microseconds(1000);
        if (coarse > clock::now())
            std::this_thread::sleep_until(coarse);
        while (clock::now() < next) { YieldProcessor(); }

        auto w = clock::now();
        jit.push_back(std::chrono::duration<double, std::micro>(w - next).count());
        next += period;
        if (next < w) next = w + period; // realign if we fell behind

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
    printf("\n[result] total ticks=%lld over %d s -> %.1f Hz\n", ticks, seconds, ticks / (double)seconds);
    printf("[result] deadline jitter us: mean=%.2f p50=%.2f p95=%.2f p99=%.2f max=%.2f\n",
           sum / jit.size(), pct(0.50), pct(0.95), pct(0.99), jit.back());
    return 0;
}
