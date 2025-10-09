#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <atomic>
#include <cstring>
#include <thread>
#include <functional>
#include <chrono>
#include <mutex>

// Plain data types in global namespace
struct ImuData {
    double roll;
    double pitch;
    double yaw;
    long long timestamp;   // nanoseconds
    int frequency;         // Hz (nominal)
    bool is_valid;
    char padding[3];
};

struct PWMData {
    int rotor1;
    int rotor2;
    int rotor3;
    int rotor4;
    long long timestamp;   // nanoseconds
    int frequency;         // Hz (nominal)
    bool is_valid;
    char padding[3];
};

struct BaroData {
    double altitude;
    long long timestamp;   // nanoseconds
    int frequency;         // Hz (nominal)
    bool is_valid;
    char padding[3];
};

struct SharedMemoryData {
    ImuData imu;
    PWMData pwm;
    BaroData baro;
    std::atomic<int> imu_sequence;
    std::atomic<int> pwm_sequence;
    std::atomic<int> baro_sequence;
    std::atomic<bool> imu_ready;
    std::atomic<bool> pwm_ready;
    std::atomic<bool> baro_ready;
    char _reserved[64];
};

class fc_memory {
public:
    fc_memory() : mem_(nullptr), handle_(nullptr) {}
    ~fc_memory() { close(); }

    // Server: create or truncate shared memory region
    bool server_create(const char* name) {
        close();
        handle_ = ::CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0,
                                       static_cast<DWORD>(sizeof(SharedMemoryData)), name);
        if (!handle_) return false;
        mem_ = static_cast<SharedMemoryData*>(::MapViewOfFile(handle_, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(SharedMemoryData)));
        if (!mem_) { ::CloseHandle(handle_); handle_ = nullptr; return false; }
        std::memset(mem_, 0, sizeof(SharedMemoryData));
        mem_->imu_sequence.store(0, std::memory_order_relaxed);
        mem_->pwm_sequence.store(0, std::memory_order_relaxed);
        mem_->baro_sequence.store(0, std::memory_order_relaxed);
        mem_->imu_ready.store(false, std::memory_order_relaxed);
        mem_->pwm_ready.store(false, std::memory_order_relaxed);
        mem_->baro_ready.store(false, std::memory_order_relaxed);
        return true;
    }

    // Client: open existing shared memory region
    bool client_connect(const char* name) {
        close();
        handle_ = ::OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, name);
        if (!handle_) return false;
        mem_ = static_cast<SharedMemoryData*>(::MapViewOfFile(handle_, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(SharedMemoryData)));
        if (!mem_) { ::CloseHandle(handle_); handle_ = nullptr; return false; }
        return true;
    }

    void close() {
        if (mem_) { ::UnmapViewOfFile(mem_); mem_ = nullptr; }
        if (handle_) { ::CloseHandle(handle_); handle_ = nullptr; }
    }

    // Server-side: publish IMU data
    void publish_imu(const ImuData& d) {
        if (!mem_) return;
        mem_->imu = d;
        mem_->imu_sequence.fetch_add(1, std::memory_order_release);
        mem_->imu_ready.store(true, std::memory_order_release);
        // update hz
        auto now = std::chrono::steady_clock::now();
        auto el = std::chrono::duration_cast<std::chrono::milliseconds>(now - imu_sec_start_).count();
        imu_cnt_sec_.fetch_add(1, std::memory_order_relaxed);
        if (el >= 1000) {
            double secs = el / 1000.0;
            imu_hz_inst_.store(static_cast<double>(imu_cnt_sec_.exchange(0)) / secs, std::memory_order_relaxed);
            imu_sec_start_ = now;
        }
    }

    // Client-side: try read PWM data
    bool try_get_pwm(PWMData& out) {
        if (!mem_) return false;
        if (!mem_->pwm_ready.load(std::memory_order_acquire)) return false;
        out = mem_->pwm;
        mem_->pwm_ready.store(false, std::memory_order_release);
        mem_->pwm_sequence.fetch_add(1, std::memory_order_release);
        // update hz
        auto now = std::chrono::steady_clock::now();
        auto el = std::chrono::duration_cast<std::chrono::milliseconds>(now - pwm_sec_start_).count();
        pwm_cnt_sec_.fetch_add(1, std::memory_order_relaxed);
        if (el >= 1000) {
            double secs = el / 1000.0;
            pwm_hz_inst_.store(static_cast<double>(pwm_cnt_sec_.exchange(0)) / secs, std::memory_order_relaxed);
            pwm_sec_start_ = now;
        }
        return true;
    }

    // Server-side: publish Baro data
    void publish_baro(const BaroData& b) {
        if (!mem_) return;
        mem_->baro = b;
        mem_->baro_sequence.fetch_add(1, std::memory_order_release);
        mem_->baro_ready.store(true, std::memory_order_release);
        auto now = std::chrono::steady_clock::now();
        auto el = std::chrono::duration_cast<std::chrono::milliseconds>(now - baro_sec_start_).count();
        baro_cnt_sec_.fetch_add(1, std::memory_order_relaxed);
        if (el >= 1000) {
            double secs = el / 1000.0;
            baro_hz_inst_.store(static_cast<double>(baro_cnt_sec_.exchange(0)) / secs, std::memory_order_relaxed);
            baro_sec_start_ = now;
        }
    }

    // Telemetry getters expected by server
    double get_imu_tx_hz() const { return imu_hz_inst_.load(std::memory_order_relaxed); }
    double get_pwm_rx_hz() const { return pwm_hz_inst_.load(std::memory_order_relaxed); }
    double get_baro_tx_hz() const { return baro_hz_inst_.load(std::memory_order_relaxed); }

    // Server worker lifecycle (no-op placeholders)
    void start_server_workers() {
        auto now = std::chrono::steady_clock::now();
        imu_sec_start_ = now; pwm_sec_start_ = now; baro_sec_start_ = now;
        imu_cnt_sec_.store(0); pwm_cnt_sec_.store(0); baro_cnt_sec_.store(0);
        imu_hz_inst_.store(0.0); pwm_hz_inst_.store(0.0); baro_hz_inst_.store(0.0);
    }
    void stop_server_workers() {}

    // Helpers expected by client/server samples
    static BaroData make_fixed_baro(long long timestamp_ns, int seq) {
        BaroData b{};
        b.altitude = 10.0;
        b.timestamp = timestamp_ns;
        b.frequency = seq;
        b.is_valid = true;
        std::memset(b.padding, 0, sizeof b.padding);
        return b;
    }

    // Client helpers used by existing client.exe
    void setup_rates(int imu_hz, int pwm_hz, int baro_hz) {
        (void)imu_hz; (void)pwm_hz; (void)baro_hz;
        // no-op: client workers poll as fast as possible
    }

    void set_client_callbacks(const std::function<void(const ImuData&)>& imu_cb,
                              const std::function<void(const BaroData&)>& baro_cb) {
        std::lock_guard<std::mutex> lk(cb_mutex_);
        imu_cb_ = imu_cb;
        baro_cb_ = baro_cb;
    }

    void start_client_workers() {
        if (running_.exchange(true)) return;
        imu_reader_ = std::thread([this]() { this->imu_loop(); });
        baro_reader_ = std::thread([this]() { this->baro_loop(); });
    }

    void stop_client_workers() {
        if (!running_.exchange(false)) return;
        if (imu_reader_.joinable()) imu_reader_.join();
        if (baro_reader_.joinable()) baro_reader_.join();
    }

    static PWMData make_fixed_pwm(long long timestamp_ns, int seq) {
        PWMData p{};
        p.rotor1 = 1000; p.rotor2 = 1000; p.rotor3 = 1000; p.rotor4 = 1000;
        p.timestamp = timestamp_ns;
        p.frequency = seq;
        p.is_valid = true;
        std::memset(p.padding, 0, sizeof p.padding);
        return p;
    }

    void client_submit_pwm(const PWMData& pwm) {
        if (!mem_) return;
        mem_->pwm = pwm;
        mem_->pwm_sequence.fetch_add(1, std::memory_order_release);
        mem_->pwm_ready.store(true, std::memory_order_release);
    }

private:
    SharedMemoryData* mem_;
    HANDLE handle_;

    std::atomic<bool> running_{false};
    std::thread imu_reader_;
    std::thread baro_reader_;
    std::mutex cb_mutex_;
    std::function<void(const ImuData&)> imu_cb_;
    std::function<void(const BaroData&)> baro_cb_;

    // telemetry
    std::chrono::steady_clock::time_point imu_sec_start_{};
    std::chrono::steady_clock::time_point pwm_sec_start_{};
    std::chrono::steady_clock::time_point baro_sec_start_{};
    std::atomic<uint64_t> imu_cnt_sec_{0};
    std::atomic<uint64_t> pwm_cnt_sec_{0};
    std::atomic<uint64_t> baro_cnt_sec_{0};
    std::atomic<double> imu_hz_inst_{0.0};
    std::atomic<double> pwm_hz_inst_{0.0};
    std::atomic<double> baro_hz_inst_{0.0};

    void imu_loop() {
        int last_seq = -1;
        while (running_.load()) {
            if (mem_) {
                int cur = mem_->imu_sequence.load(std::memory_order_acquire);
                if (cur != last_seq && mem_->imu_ready.load(std::memory_order_acquire)) {
                    ImuData d = mem_->imu;
                    last_seq = cur;
                    std::function<void(const ImuData&)> cb;
                    {
                        std::lock_guard<std::mutex> lk(cb_mutex_);
                        cb = imu_cb_;
                    }
                    if (cb) cb(d);
                }
            }
            // tight spin is acceptable for test client; add small yield
            std::this_thread::yield();
        }
    }

    void baro_loop() {
        int last_seq = -1;
        while (running_.load()) {
            if (mem_) {
                int cur = mem_->baro_sequence.load(std::memory_order_acquire);
                if (cur != last_seq && mem_->baro_ready.load(std::memory_order_acquire)) {
                    BaroData b = mem_->baro;
                    last_seq = cur;
                    std::function<void(const BaroData&)> cb;
                    {
                        std::lock_guard<std::mutex> lk(cb_mutex_);
                        cb = baro_cb_;
                    }
                    if (cb) cb(b);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
};
