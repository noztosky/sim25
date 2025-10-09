#pragma once
#include <windows.h>
#include <string>
#include <cstring>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include "x_pipeline.h"
#include <functional>
#include <queue>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

struct ImuData {
    double roll;
    double pitch;
    double yaw;
    long long timestamp;
    int frequency;
    bool is_valid;
    char padding[3];
};

struct BaroData {
    double altitude;
    long long timestamp;
    int frequency;
    bool is_valid;
    char padding[3];
};

struct PWMData {
    int rotor1;
    int rotor2;
    int rotor3;
    int rotor4;
    long long timestamp;
    int frequency;
    bool is_valid;
    char padding[3];
};

class fc_pipeline {
public:
    fc_pipeline()
        : imu_event_(NULL), imu_write_inflight_(false), last_imu_bytes_written_(0),
          pwm_event_(NULL), pwm_read_inflight_(false), last_pwm_bytes_read_(0),
          baro_event_(NULL), baro_write_inflight_(false), last_baro_bytes_written_(0) {
        ZeroMemory(&imu_overlapped_, sizeof(imu_overlapped_));
        ZeroMemory(&pwm_overlapped_, sizeof(pwm_overlapped_));
        ZeroMemory(&baro_overlapped_, sizeof(baro_overlapped_));
        server_running_.store(false);
        imu_rate_hz_.store(0);
        pwm_rate_hz_.store(0);
        baro_rate_hz_.store(0);
        imu_tx_count_.store(0);
        pwm_rx_count_.store(0);
        baro_tx_count_.store(0);
        window_start_tp_ = std::chrono::steady_clock::now();
        imu_sec_start_tp_ = window_start_tp_;
        pwm_sec_start_tp_ = window_start_tp_;
        baro_sec_start_tp_ = window_start_tp_;
        imu_tx_cnt_sec_.store(0);
        pwm_rx_cnt_sec_.store(0);
        baro_tx_cnt_sec_.store(0);
        imu_hz_inst_.store(0.0);
        pwm_hz_inst_.store(0.0);
        baro_hz_inst_.store(0.0);
        pwm_new_.store(false);
        latest_imu_ = {};
        latest_baro_ = {};
        latest_pwm_ = {};
    }

    ~fc_pipeline() {
        close();
    }

    // Server-side
    bool server_create(const char* imu_name, const char* pwm_name, const char* baro_name) {
        // Increase buffer sizes for high-frequency data (256KB each)
        const DWORD large_buffer = 262144; // 256KB
        if (!imu_pipe_.create_server(imu_name,
                                     PIPE_ACCESS_OUTBOUND | FILE_FLAG_OVERLAPPED,
                                     PIPE_TYPE_BYTE | PIPE_WAIT,
                                     large_buffer, large_buffer)) return false;
        if (!pwm_pipe_.create_server(pwm_name,
                                     PIPE_ACCESS_INBOUND | FILE_FLAG_OVERLAPPED,
                                     PIPE_TYPE_BYTE | PIPE_NOWAIT,
                                     large_buffer, large_buffer)) {
            imu_pipe_.close();
            return false;
        }
        if (!baro_pipe_.create_server(baro_name,
                                      PIPE_ACCESS_OUTBOUND | FILE_FLAG_OVERLAPPED,
                                      PIPE_TYPE_BYTE | PIPE_WAIT,
                                      large_buffer, large_buffer)) {
            imu_pipe_.close();
            pwm_pipe_.close();
            return false;
        }
        if (imu_event_ == NULL) {
            imu_event_ = CreateEvent(NULL, 1, 0, NULL);
        }
        if (pwm_event_ == NULL) {
            pwm_event_ = CreateEvent(NULL, 1, 0, NULL);
        }
        if (baro_event_ == NULL) {
            baro_event_ = CreateEvent(NULL, 1, 0, NULL);
        }
        ZeroMemory(&imu_overlapped_, sizeof(imu_overlapped_));
        imu_overlapped_.hEvent = imu_event_;
        imu_write_inflight_ = false;
        last_imu_bytes_written_ = 0;
        
        ZeroMemory(&pwm_overlapped_, sizeof(pwm_overlapped_));
        pwm_overlapped_.hEvent = pwm_event_;
        pwm_read_inflight_ = false;
        last_pwm_bytes_read_ = 0;
        
        ZeroMemory(&baro_overlapped_, sizeof(baro_overlapped_));
        baro_overlapped_.hEvent = baro_event_;
        baro_write_inflight_ = false;
        last_baro_bytes_written_ = 0;
        return true;
    }

    bool server_wait_connections(bool* /*pwm_listening*/ = nullptr) {
        return true; // Non-blocking; rely on first successful I/O
    }

    // Non-blocking IMU write: poll completion, then start new write if possible
    // Returns: 0=no action, 1=completed, 2=started, -1=error
    int imu_write_step(const ImuData& data, DWORD* out_completed_bytes = nullptr) {
        if (imu_write_inflight_) {
            DWORD completed_bytes = 0;
            if (GetOverlappedResult(imu_pipe_.get_handle(), &imu_overlapped_, &completed_bytes, 0) != 0) {
                imu_write_inflight_ = false;
                last_imu_bytes_written_ = completed_bytes;
                if (out_completed_bytes) *out_completed_bytes = completed_bytes;
                return 1; // completed
            }
        }
        if (!imu_write_inflight_) {
            if (imu_event_ != NULL) ResetEvent(imu_event_);
            ZeroMemory(&imu_overlapped_, sizeof(imu_overlapped_));
            imu_overlapped_.hEvent = imu_event_;
            DWORD immediate = 0;
            BOOL ok = WriteFile(imu_pipe_.get_handle(), &data, sizeof(ImuData), &immediate, &imu_overlapped_);
            if (ok == 0) {
                DWORD err = GetLastError();
                if (err == ERROR_IO_PENDING) {
                    imu_write_inflight_ = true;
                    return 2; // started
                }
                if (err == ERROR_NO_DATA ||
                    err == ERROR_PIPE_LISTENING ||
                    err == ERROR_PIPE_NOT_CONNECTED ||
                    err == ERROR_BAD_PIPE ||
                    err == ERROR_BROKEN_PIPE) {
                    return 0; // not connected/temporarily unavailable
                }
                return -1; // other errors are fatal
            } else {
                last_imu_bytes_written_ = immediate;
                if (out_completed_bytes) *out_completed_bytes = immediate;
                return 1; // completed immediately
            }
        }
        return 0;
    }

    // Non-blocking PWM read: poll completion, then start new read if possible
    // Returns: 0=no action, 1=completed, 2=started, -1=error
    int pwm_read_step(PWMData& out, DWORD* out_completed_bytes = nullptr) {
        if (pwm_read_inflight_) {
            DWORD completed_bytes = 0;
            if (GetOverlappedResult(pwm_pipe_.get_handle(), &pwm_overlapped_, &completed_bytes, 0) != 0) {
                pwm_read_inflight_ = false;
                last_pwm_bytes_read_ = completed_bytes;
                if (out_completed_bytes) *out_completed_bytes = completed_bytes;
                return 1; // completed
            }
        }
        if (!pwm_read_inflight_) {
            if (pwm_event_ != NULL) ResetEvent(pwm_event_);
            ZeroMemory(&pwm_overlapped_, sizeof(pwm_overlapped_));
            pwm_overlapped_.hEvent = pwm_event_;
            DWORD immediate = 0;
            BOOL ok = ReadFile(pwm_pipe_.get_handle(), &out, sizeof(PWMData), &immediate, &pwm_overlapped_);
            if (ok == 0) {
                DWORD err = GetLastError();
                if (err == ERROR_IO_PENDING) {
                    pwm_read_inflight_ = true;
                    return 2; // started
                }
                if (err == ERROR_NO_DATA ||
                    err == ERROR_PIPE_LISTENING ||
                    err == ERROR_PIPE_NOT_CONNECTED ||
                    err == ERROR_BAD_PIPE ||
                    err == ERROR_BROKEN_PIPE) {
                    return 0; // not connected/temporarily unavailable
                }
                return -1; // other errors are fatal
            } else {
                last_pwm_bytes_read_ = immediate;
                if (out_completed_bytes) *out_completed_bytes = immediate;
                return 1; // completed immediately
            }
        }
        return 0;
    }

    // Non-blocking BARO write: poll completion, then start new write if possible
    // Returns: 0=no action, 1=completed, 2=started, -1=error
    int baro_write_step(const BaroData& data, DWORD* out_completed_bytes = nullptr) {
        if (baro_write_inflight_) {
            DWORD completed_bytes = 0;
            if (GetOverlappedResult(baro_pipe_.get_handle(), &baro_overlapped_, &completed_bytes, 0) != 0) {
                baro_write_inflight_ = false;
                last_baro_bytes_written_ = completed_bytes;
                if (out_completed_bytes) *out_completed_bytes = completed_bytes;
                return 1; // completed
            }
        }
        if (!baro_write_inflight_) {
            if (baro_event_ != NULL) ResetEvent(baro_event_);
            ZeroMemory(&baro_overlapped_, sizeof(baro_overlapped_));
            baro_overlapped_.hEvent = baro_event_;
            DWORD immediate = 0;
            BOOL ok = WriteFile(baro_pipe_.get_handle(), &data, sizeof(BaroData), &immediate, &baro_overlapped_);
            if (ok == 0) {
                DWORD err = GetLastError();
                if (err == ERROR_IO_PENDING) {
                    baro_write_inflight_ = true;
                    return 2; // started
                }
                if (err == ERROR_NO_DATA ||
                    err == ERROR_PIPE_LISTENING ||
                    err == ERROR_PIPE_NOT_CONNECTED ||
                    err == ERROR_BAD_PIPE ||
                    err == ERROR_BROKEN_PIPE) {
                    return 0; // not connected/temporarily unavailable
                }
                return -1; // other errors are fatal
            } else {
                last_baro_bytes_written_ = immediate;
                if (out_completed_bytes) *out_completed_bytes = immediate;
                return 1; // completed immediately
            }
        }
        return 0;
    }

    bool server_read_pwm(PWMData& out, DWORD* bytes_read = nullptr) {
        DWORD br = 0;
        if (!pwm_pipe_.read_sync(&out, sizeof(PWMData), &br)) {
            return false;
        }
        if (bytes_read) *bytes_read = br;
        return true;
    }

    bool server_write_baro_sync(const BaroData& data, DWORD* bytes_written = nullptr) {
        DWORD bw = 0;
        if (!baro_pipe_.write_sync(&data, sizeof(BaroData), &bw)) return false;
        if (bytes_written) *bytes_written = bw;
        return true;
    }

    // Client-side
    bool client_connect(const char* imu_name, const char* pwm_name) {
        if (!imu_pipe_.open_client(imu_name, GENERIC_READ)) return false;
        if (!pwm_pipe_.open_client(pwm_name, GENERIC_WRITE)) {
            imu_pipe_.close();
            return false;
        }
        return true;
    }

    bool client_connect_with_baro(const char* imu_name, const char* pwm_name, const char* baro_name) {
        if (!imu_pipe_.open_client(imu_name, GENERIC_READ)) return false;
        if (!pwm_pipe_.open_client(pwm_name, GENERIC_WRITE)) {
            imu_pipe_.close();
            return false;
        }
        // Try BARO but don't fail whole connect if baro is absent
        (void)baro_pipe_.open_client(baro_name, GENERIC_READ);
        return true;
    }

    bool client_read_imu(ImuData& out, DWORD* bytes_read = nullptr) {
        DWORD br = 0;
        if (!imu_pipe_.read_sync(&out, sizeof(ImuData), &br)) return false;
        if (bytes_read) *bytes_read = br;
        return true;
    }

    bool client_send_pwm(const PWMData& in, DWORD* bytes_written = nullptr) {
        DWORD bw = 0;
        if (!pwm_pipe_.write_sync(&in, sizeof(PWMData), &bw)) return false;
        if (bytes_written) *bytes_written = bw;
        return true;
    }

    bool client_read_baro(BaroData& out, DWORD* bytes_read = nullptr) {
        DWORD br = 0;
        if (!baro_pipe_.read_sync(&out, sizeof(BaroData), &br)) return false;
        if (bytes_read) *bytes_read = br;
        return true;
    }

    void close() {
        stop_server_workers();
        stop_client_workers();
        if (imu_event_ != NULL) {
            CloseHandle(imu_event_);
            imu_event_ = NULL;
        }
        if (pwm_event_ != NULL) {
            CloseHandle(pwm_event_);
            pwm_event_ = NULL;
        }
        if (baro_event_ != NULL) {
            CloseHandle(baro_event_);
            baro_event_ = NULL;
        }
        imu_pipe_.close();
        pwm_pipe_.close();
        baro_pipe_.close();
        imu_write_inflight_ = false;
        last_imu_bytes_written_ = 0;
        pwm_read_inflight_ = false;
        last_pwm_bytes_read_ = 0;
        baro_write_inflight_ = false;
        last_baro_bytes_written_ = 0;
        ZeroMemory(&imu_overlapped_, sizeof(imu_overlapped_));
        ZeroMemory(&pwm_overlapped_, sizeof(pwm_overlapped_));
        ZeroMemory(&baro_overlapped_, sizeof(baro_overlapped_));
    }

    bool imu_inflight() const { return imu_write_inflight_; }
    DWORD last_imu_bytes() const { return last_imu_bytes_written_; }
    
    bool pwm_inflight() const { return pwm_read_inflight_; }
    DWORD last_pwm_bytes() const { return last_pwm_bytes_read_; }
    
    bool baro_inflight() const { return baro_write_inflight_; }
    DWORD last_baro_bytes() const { return last_baro_bytes_written_; }

    // Helpers for fixed dummy data
    static ImuData make_fixed_imu(long long timestamp_ns, int freq_counter) {
        ImuData d;
        d.roll = 1.0;
        d.pitch = 2.0;
        d.yaw = 3.0;
        d.timestamp = timestamp_ns;
        d.frequency = freq_counter;
        d.is_valid = true;
        std::memset(d.padding, 0, sizeof d.padding);
        return d;
    }

    static PWMData make_fixed_pwm(long long timestamp_ns, int freq_counter) {
        PWMData p;
        p.rotor1 = 1000;
        p.rotor2 = 2000;
        p.rotor3 = 3000;
        p.rotor4 = 4000;
        p.timestamp = timestamp_ns;
        p.frequency = freq_counter;
        p.is_valid = true;
        std::memset(p.padding, 0, sizeof p.padding);
        return p;
    }

    static BaroData make_fixed_baro(long long timestamp_ns, int freq_counter) {
        BaroData b;
        b.altitude = 10.0;
        b.timestamp = timestamp_ns;
        b.frequency = freq_counter;
        b.is_valid = true;
        std::memset(b.padding, 0, sizeof b.padding);
        return b;
    }

    // Background workers (optional use)
    void setup_rates(int imu_hz, int pwm_hz, int baro_hz) {
        imu_rate_hz_.store(imu_hz);
        pwm_rate_hz_.store(pwm_hz);
        baro_rate_hz_.store(baro_hz);
    }

    void start_server_workers() {
        if (server_running_.exchange(true)) return;
        enable_timer_granularity();
        window_start_tp_ = std::chrono::steady_clock::now();
        // ensure server pipes are accepting connections
        async_connect(imu_pipe_, imu_connecting_);
        async_connect(pwm_pipe_, pwm_connecting_);
        async_connect(baro_pipe_, baro_connecting_);
        server_imu_writer_ = std::thread([this]() { run_server_imu_writer(); });
        server_baro_writer_ = std::thread([this]() { run_server_baro_writer(); });
        server_pwm_reader_ = std::thread([this]() { run_server_pwm_reader(); });
    }

    void stop_server_workers() {
        if (!server_running_.exchange(false)) return;
        if (server_imu_writer_.joinable()) server_imu_writer_.join();
        if (server_baro_writer_.joinable()) server_baro_writer_.join();
        if (server_pwm_reader_.joinable()) server_pwm_reader_.join();
        disable_timer_granularity();
    }

    // Client-side background workers (optional)
    void set_client_callbacks(const std::function<void(const ImuData&)>& imu_cb,
                              const std::function<void(const BaroData&)>& baro_cb) {
        std::lock_guard<std::mutex> lk(cb_mutex_);
        imu_callback_ = imu_cb;
        baro_callback_ = baro_cb;
    }

    void start_client_workers() {
        if (client_running_.exchange(true)) return;
        enable_timer_granularity();
        client_imu_reader_ = std::thread([this]() { run_client_imu_reader(); });
        client_baro_reader_ = std::thread([this]() { run_client_baro_reader(); });
        client_pwm_writer_ = std::thread([this]() { run_client_pwm_writer(); });
    }

    void stop_client_workers() {
        if (!client_running_.exchange(false)) return;
        pwmq_cv_.notify_all();
        if (client_imu_reader_.joinable()) client_imu_reader_.join();
        if (client_baro_reader_.joinable()) client_baro_reader_.join();
        if (client_pwm_writer_.joinable()) client_pwm_writer_.join();
        disable_timer_granularity();
        // drain queue
        {
            std::lock_guard<std::mutex> ql(pwmq_mutex_);
            while (!pwm_queue_.empty()) pwm_queue_.pop();
        }
    }

    void client_submit_pwm(const PWMData& pwm) {
        {
            std::lock_guard<std::mutex> ql(pwmq_mutex_);
            pwm_queue_.push(pwm);
        }
        pwmq_cv_.notify_one();
    }

    // Provide freshest data to workers
    void publish_imu(const ImuData& data) {
        std::lock_guard<std::mutex> lk(data_mutex_);
        latest_imu_ = data;
    }

    void publish_baro(const BaroData& data) {
        std::lock_guard<std::mutex> lk(data_mutex_);
        latest_baro_ = data;
    }

    // Non-blocking fetch of latest received PWM
    bool try_get_pwm(PWMData& out) {
        if (!pwm_new_.exchange(false)) return false;
        std::lock_guard<std::mutex> lk(pwm_mutex_);
        out = latest_pwm_;
        return true;
    }

    // Telemetry: actual I/O rates since start_server_workers()
    // Instantaneous (sliding ~1s) rates
    double get_imu_tx_hz() const { return imu_hz_inst_.load(); }
    double get_pwm_rx_hz() const { return pwm_hz_inst_.load(); }
    double get_baro_tx_hz() const { return baro_hz_inst_.load(); }

private:
    x_pipeline imu_pipe_;
    x_pipeline pwm_pipe_;
    x_pipeline baro_pipe_;
    HANDLE imu_event_;
    OVERLAPPED imu_overlapped_;
    bool imu_write_inflight_;
    DWORD last_imu_bytes_written_;
    
    HANDLE pwm_event_;
    OVERLAPPED pwm_overlapped_;
    bool pwm_read_inflight_;
    DWORD last_pwm_bytes_read_;
    
    HANDLE baro_event_;
    OVERLAPPED baro_overlapped_;
    bool baro_write_inflight_;
    DWORD last_baro_bytes_written_;

    // Workers and state
    std::thread server_imu_writer_;
    std::thread server_baro_writer_;
    std::thread server_pwm_reader_;
    std::atomic<bool> server_running_ { false };
    std::atomic<int> imu_rate_hz_ { 0 };
    std::atomic<int> pwm_rate_hz_ { 0 };
    std::atomic<int> baro_rate_hz_ { 0 };

    mutable std::mutex data_mutex_;
    ImuData latest_imu_{};
    BaroData latest_baro_{};

    mutable std::mutex pwm_mutex_;
    PWMData latest_pwm_{};
    std::atomic<bool> pwm_new_ { false };

    std::atomic<uint64_t> imu_tx_count_ { 0 };
    std::atomic<uint64_t> pwm_rx_count_ { 0 };
    std::atomic<uint64_t> baro_tx_count_ { 0 };
    std::chrono::steady_clock::time_point window_start_tp_;
    // per-channel 1s windows
    std::chrono::steady_clock::time_point imu_sec_start_tp_;
    std::chrono::steady_clock::time_point pwm_sec_start_tp_;
    std::chrono::steady_clock::time_point baro_sec_start_tp_;
    std::atomic<uint64_t> imu_tx_cnt_sec_ { 0 };
    std::atomic<uint64_t> pwm_rx_cnt_sec_ { 0 };
    std::atomic<uint64_t> baro_tx_cnt_sec_ { 0 };
    std::atomic<double> imu_hz_inst_ { 0.0 };
    std::atomic<double> pwm_hz_inst_ { 0.0 };
    std::atomic<double> baro_hz_inst_ { 0.0 };

    // Client worker state
    std::thread client_imu_reader_;
    std::thread client_baro_reader_;
    std::thread client_pwm_writer_;
    std::atomic<bool> client_running_ { false };
    std::function<void(const ImuData&)> imu_callback_;
    std::function<void(const BaroData&)> baro_callback_;
    std::mutex cb_mutex_;
    std::mutex pwmq_mutex_;
    std::condition_variable pwmq_cv_;
    std::queue<PWMData> pwm_queue_;
    std::atomic<bool> imu_connecting_ { false };
    std::atomic<bool> pwm_connecting_ { false };
    std::atomic<bool> baro_connecting_ { false };

    void run_server_imu_writer() {
        while (server_running_.load()) {
            const int hz = imu_rate_hz_.load();
            const auto sleep_us = hz > 0 ? (1000000 / hz) : 10000;
            ImuData to_send{};
            {
                std::lock_guard<std::mutex> lk(data_mutex_);
                to_send = latest_imu_;
            }
            if (to_send.is_valid && imu_pipe_.is_valid()) {
                DWORD bw = 0;
                if (imu_pipe_.write_sync(&to_send, sizeof(ImuData), &bw) && bw == sizeof(ImuData)) {
                    imu_tx_count_.fetch_add(1);
                    imu_tx_cnt_sec_.fetch_add(1);
                    auto now = std::chrono::steady_clock::now();
                    auto el = std::chrono::duration_cast<std::chrono::milliseconds>(now - imu_sec_start_tp_).count();
                    if (el >= 1000) {
                        double secs = el / 1000.0;
                        double hz_calc = static_cast<double>(imu_tx_cnt_sec_.exchange(0)) / secs;
                        imu_hz_inst_.store(hz_calc);
                        imu_sec_start_tp_ = now;
                    }
                } else {
                    DWORD err = GetLastError();
                    if (err == ERROR_NO_DATA || err == ERROR_PIPE_LISTENING || err == ERROR_PIPE_NOT_CONNECTED || err == ERROR_BAD_PIPE || err == ERROR_BROKEN_PIPE) {
                        if (!imu_connecting_.exchange(true)) {
                            std::thread([this]() { this->reconnect_server_pipe(this->imu_pipe_, this->imu_connecting_); }).detach();
                        }
                    }
                }
            }
            sleep_precise_us(sleep_us);
        }
    }

    void run_server_baro_writer() {
        while (server_running_.load()) {
            const int hz = baro_rate_hz_.load();
            const auto sleep_us = hz > 0 ? (1000000 / hz) : 20000;
            BaroData to_send{};
            {
                std::lock_guard<std::mutex> lk(data_mutex_);
                to_send = latest_baro_;
            }
            if (baro_pipe_.is_valid()) {
                DWORD bw = 0;
                if (to_send.is_valid && baro_pipe_.write_sync(&to_send, sizeof(BaroData), &bw) && bw == sizeof(BaroData)) {
                    baro_tx_count_.fetch_add(1);
                    baro_tx_cnt_sec_.fetch_add(1);
                    auto now = std::chrono::steady_clock::now();
                    auto el = std::chrono::duration_cast<std::chrono::milliseconds>(now - baro_sec_start_tp_).count();
                    if (el >= 1000) {
                        double secs = el / 1000.0;
                        double hz_calc = static_cast<double>(baro_tx_cnt_sec_.exchange(0)) / secs;
                        baro_hz_inst_.store(hz_calc);
                        baro_sec_start_tp_ = now;
                    }
                } else {
                    DWORD err = GetLastError();
                    if (err == ERROR_NO_DATA || err == ERROR_PIPE_LISTENING || err == ERROR_PIPE_NOT_CONNECTED || err == ERROR_BAD_PIPE || err == ERROR_BROKEN_PIPE) {
                        if (!baro_connecting_.exchange(true)) {
                            std::thread([this]() { this->reconnect_server_pipe(this->baro_pipe_, this->baro_connecting_); }).detach();
                        }
                    }
                }
            }
            sleep_precise_us(sleep_us);
        }
    }

    void run_server_pwm_reader() {
        while (server_running_.load()) {
            const int hz = pwm_rate_hz_.load();
            const auto sleep_us = hz > 0 ? (1000000 / hz) : 5000;
            if (pwm_pipe_.is_valid()) {
                DWORD avail = 0;
                if (PeekNamedPipe(pwm_pipe_.get_handle(), NULL, 0, NULL, &avail, NULL) && avail >= sizeof(PWMData)) {
                    PWMData in{};
                    DWORD br = 0;
                    if (pwm_pipe_.read_sync(&in, sizeof(PWMData), &br) && br == sizeof(PWMData) && in.is_valid) {
                        std::lock_guard<std::mutex> lk(pwm_mutex_);
                        latest_pwm_ = in;
                        pwm_new_.store(true);
                        pwm_rx_count_.fetch_add(1);
                        pwm_rx_cnt_sec_.fetch_add(1);
                        auto now = std::chrono::steady_clock::now();
                        auto el = std::chrono::duration_cast<std::chrono::milliseconds>(now - pwm_sec_start_tp_).count();
                        if (el >= 1000) {
                            double secs = el / 1000.0;
                            double hz_calc = static_cast<double>(pwm_rx_cnt_sec_.exchange(0)) / secs;
                            pwm_hz_inst_.store(hz_calc);
                            pwm_sec_start_tp_ = now;
                        }
                    }
                } else {
                    DWORD err = GetLastError();
                    if (err == ERROR_NO_DATA || err == ERROR_PIPE_LISTENING || err == ERROR_PIPE_NOT_CONNECTED || err == ERROR_BAD_PIPE || err == ERROR_BROKEN_PIPE) {
                        if (!pwm_connecting_.exchange(true)) {
                            std::thread([this]() { this->reconnect_server_pipe(this->pwm_pipe_, this->pwm_connecting_); }).detach();
                        }
                    }
                }
            }
            sleep_precise_us(sleep_us);
        }
    }

    double calc_hz(uint64_t count) const {
        const auto now = std::chrono::steady_clock::now();
        const double secs = std::chrono::duration_cast<std::chrono::milliseconds>(now - window_start_tp_).count() / 1000.0;
        if (secs <= 0.0) return 0.0;
        return static_cast<double>(count) / secs;
    }

    void run_client_imu_reader() {
        while (client_running_.load()) {
            if (imu_pipe_.is_valid()) {
                DWORD avail = 0;
                if (PeekNamedPipe(imu_pipe_.get_handle(), NULL, 0, NULL, &avail, NULL) && avail >= sizeof(ImuData)) {
                    ImuData in{};
                    DWORD br = 0;
                    if (imu_pipe_.read_sync(&in, sizeof(ImuData), &br) && br == sizeof(ImuData) && in.is_valid) {
                        // update 1s window instant Hz on client (IMU receive)
                        imu_tx_count_.fetch_add(1);
                        imu_tx_cnt_sec_.fetch_add(1);
                        auto now = std::chrono::steady_clock::now();
                        auto el = std::chrono::duration_cast<std::chrono::milliseconds>(now - imu_sec_start_tp_).count();
                        if (el >= 1000) {
                            double secs = el / 1000.0;
                            double hz = static_cast<double>(imu_tx_cnt_sec_.exchange(0)) / secs;
                            imu_hz_inst_.store(hz);
                            imu_sec_start_tp_ = now;
                        }
                        std::function<void(const ImuData&)> cb;
                        {
                            std::lock_guard<std::mutex> lk(cb_mutex_);
                            cb = imu_callback_;
                        }
                        if (cb) cb(in);
                    }
                }
            }
            const int hz = imu_rate_hz_.load();
            const auto sleep_us = hz > 0 ? (1000000 / hz) : 5000;
            sleep_precise_us(sleep_us);
        }
    }

    void run_client_baro_reader() {
        while (client_running_.load()) {
            if (baro_pipe_.is_valid()) {
                DWORD avail = 0;
                if (PeekNamedPipe(baro_pipe_.get_handle(), NULL, 0, NULL, &avail, NULL) && avail >= sizeof(BaroData)) {
                    BaroData in{};
                    DWORD br = 0;
                    if (baro_pipe_.read_sync(&in, sizeof(BaroData), &br) && br == sizeof(BaroData) && in.is_valid) {
                        // update 1s window instant Hz on client (BARO receive)
                        baro_tx_count_.fetch_add(1);
                        baro_tx_cnt_sec_.fetch_add(1);
                        auto now = std::chrono::steady_clock::now();
                        auto el = std::chrono::duration_cast<std::chrono::milliseconds>(now - baro_sec_start_tp_).count();
                        if (el >= 1000) {
                            double secs = el / 1000.0;
                            double hz = static_cast<double>(baro_tx_cnt_sec_.exchange(0)) / secs;
                            baro_hz_inst_.store(hz);
                            baro_sec_start_tp_ = now;
                        }
                        std::function<void(const BaroData&)> cb;
                        {
                            std::lock_guard<std::mutex> lk(cb_mutex_);
                            cb = baro_callback_;
                        }
                        if (cb) cb(in);
                    }
                }
            }
            const int hz = baro_rate_hz_.load();
            const auto sleep_us = hz > 0 ? (1000000 / hz) : 10000;
            sleep_precise_us(sleep_us);
        }
    }

    void run_client_pwm_writer() {
        while (client_running_.load()) {
            PWMData out{};
            bool has = false;
            {
                std::unique_lock<std::mutex> ul(pwmq_mutex_);
                pwmq_cv_.wait_for(ul, std::chrono::milliseconds(10), [this]() { return !pwm_queue_.empty() || !client_running_.load(); });
                if (!pwm_queue_.empty()) { out = pwm_queue_.front(); pwm_queue_.pop(); has = true; }
            }
            if (has && pwm_pipe_.is_valid()) {
                DWORD bw = 0;
                if (pwm_pipe_.write_sync(&out, sizeof(PWMData), &bw) && bw == sizeof(PWMData)) {
                    // update 1s window instant Hz on client (PWM transmit)
                    pwm_rx_count_.fetch_add(1);
                    pwm_rx_cnt_sec_.fetch_add(1);
                    auto now = std::chrono::steady_clock::now();
                    auto el = std::chrono::duration_cast<std::chrono::milliseconds>(now - pwm_sec_start_tp_).count();
                    if (el >= 1000) {
                        double secs = el / 1000.0;
                        double hz = static_cast<double>(pwm_rx_cnt_sec_.exchange(0)) / secs;
                        pwm_hz_inst_.store(hz);
                        pwm_sec_start_tp_ = now;
                    }
                } else {
                    DWORD err = GetLastError();
                    if (err == ERROR_NO_DATA || err == ERROR_PIPE_LISTENING || err == ERROR_PIPE_NOT_CONNECTED || err == ERROR_BAD_PIPE || err == ERROR_BROKEN_PIPE) {
                        // try reconnect client-side write pipe
                        (void)pwm_pipe_.open_client(pwm_pipe_.name().c_str(), GENERIC_WRITE);
                    }
                }
            }
        }
    }

    // Attempt sub-ms sleep using a hybrid of Sleep and spin/yield for precision
    static void sleep_precise_us(int total_us) {
        if (total_us <= 0) return;
        auto target = std::chrono::high_resolution_clock::now() + std::chrono::microseconds(total_us);
        for (;;) {
            auto now = std::chrono::high_resolution_clock::now();
            if (now >= target) break;
            auto remain_us = std::chrono::duration_cast<std::chrono::microseconds>(target - now).count();
            if (remain_us > 2000) {
                Sleep(static_cast<DWORD>(remain_us / 1000 - 1));
            } else {
                std::this_thread::yield();
            }
        }
    }

    static void enable_timer_granularity() {
        static std::atomic<long> users(0);
        if (users.fetch_add(1) == 0) {
            timeBeginPeriod(1);
        }
    }

    static void disable_timer_granularity() {
        static std::atomic<long> users(0); // same static as above
        long prev = users.load();
        while (prev > 0 && !users.compare_exchange_weak(prev, prev - 1)) {}
        if (prev == 1) timeEndPeriod(1);
    }

    void async_connect(x_pipeline& pipe, std::atomic<bool>& flag) {
        if (flag.exchange(true)) return;
        std::thread([this, &pipe, &flag]() {
            BOOL immediate = 0; DWORD last = 0;
            (void)pipe.connect_server(&immediate, &last);
            flag.store(false);
        }).detach();
    }

    void reconnect_server_pipe(x_pipeline& pipe, std::atomic<bool>& flag) {
        pipe.disconnect();
        BOOL immediate = 0; DWORD last = 0;
        (void)pipe.connect_server(&immediate, &last);
        flag.store(false);
    }
};


