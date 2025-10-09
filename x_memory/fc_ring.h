#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")
#include <atomic>
#include <cstring>
#include <thread>
#include <functional>
#include <chrono>
#include <mutex>

// Plain data types (same as fc_memory)
struct ImuData {
    double roll;
    double pitch;
    double yaw;
    long long timestamp;   // ns
    int frequency;         // seq or nominal hz
    bool is_valid;
    char padding[3];
};
struct PWMData {
    int rotor1; int rotor2; int rotor3; int rotor4;
    long long timestamp; int frequency; bool is_valid; char padding[3];
};
struct BaroData {
    double altitude; long long timestamp; int frequency; bool is_valid; char padding[3];
};

// SPSC ring-buffer layout for IMU
struct ImuSlot { ImuData data; uint32_t seq; };

template <size_t CAP>
struct SharedRingData {
    // IMU ring
    ImuSlot imu_slots[CAP];
    std::atomic<uint32_t> imu_head_seq; // last produced seq
    // single-slot others (kept for compatibility)
    PWMData pwm; std::atomic<int> pwm_sequence; std::atomic<bool> pwm_ready;
    BaroData baro; std::atomic<int> baro_sequence; std::atomic<bool> baro_ready;
};

class fc_ring {
public:
    static constexpr size_t CAPACITY = 16; // ~16 ms @1kHz
    using Shared = SharedRingData<CAPACITY>;

    fc_ring(): mem_(nullptr), handle_(nullptr) {}
    ~fc_ring(){ close(); }

    bool server_create(const char* name){
        close();
        handle_ = ::CreateFileMappingA(INVALID_HANDLE_VALUE,nullptr,PAGE_READWRITE,0,(DWORD)sizeof(Shared),name);
        if(!handle_) return false;
        mem_ = static_cast<Shared*>(::MapViewOfFile(handle_,FILE_MAP_ALL_ACCESS,0,0,sizeof(Shared)));
        if(!mem_){::CloseHandle(handle_); handle_=nullptr; return false;}
        std::memset(mem_,0,sizeof(Shared));
        mem_->imu_head_seq.store(0,std::memory_order_relaxed);
        mem_->pwm_sequence.store(0,std::memory_order_relaxed); mem_->pwm_ready.store(false,std::memory_order_relaxed);
        mem_->baro_sequence.store(0,std::memory_order_relaxed); mem_->baro_ready.store(false,std::memory_order_relaxed);
        return true;
    }
    bool client_connect(const char* name){
        close();
        handle_ = ::OpenFileMappingA(FILE_MAP_ALL_ACCESS,FALSE,name);
        if(!handle_) return false;
        mem_ = static_cast<Shared*>(::MapViewOfFile(handle_,FILE_MAP_ALL_ACCESS,0,0,sizeof(Shared)));
        if(!mem_){::CloseHandle(handle_); handle_=nullptr; return false;}
        return true;
    }
    void close(){ if(mem_){::UnmapViewOfFile(mem_); mem_=nullptr;} if(handle_){::CloseHandle(handle_); handle_=nullptr;} }

    // Producer API (server/UE)
    void publish_imu(const ImuData& d){
        if(!mem_) return;
        uint32_t next = mem_->imu_head_seq.load(std::memory_order_relaxed)+1;
        size_t idx = next % CAPACITY;
        mem_->imu_slots[idx].data = d;
        mem_->imu_slots[idx].seq = next;
        mem_->imu_head_seq.store(next,std::memory_order_release);
        // server-side telemetry
        imu_cnt_.fetch_add(1, std::memory_order_relaxed);
        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - imu_start_).count();
        if(ms >= 1000){
            double secs = ms / 1000.0;
            imu_hz_.store(double(imu_cnt_.exchange(0)) / secs, std::memory_order_relaxed);
            imu_start_ = now;
        }
    }
    void publish_baro(const BaroData& b){
        if(!mem_) return;
        mem_->baro = b;
        mem_->baro_sequence.fetch_add(1,std::memory_order_release);
        mem_->baro_ready.store(true,std::memory_order_release);
        // server-side telemetry
        baro_cnt_.fetch_add(1, std::memory_order_relaxed);
        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - baro_start_).count();
        if(ms >= 1000){
            double secs = ms / 1000.0;
            baro_hz_.store(double(baro_cnt_.exchange(0)) / secs, std::memory_order_relaxed);
            baro_start_ = now;
        }
    }
    void client_submit_pwm(const PWMData& p){
        if(!mem_) return;
        mem_->pwm = p;
        mem_->pwm_sequence.fetch_add(1,std::memory_order_release);
        mem_->pwm_ready.store(true,std::memory_order_release);
    }

    // Consumer helpers
    template<typename Fn>
    void consume_new_imu(uint32_t& last_seq, Fn&& on_sample){
        if(!mem_) return; uint32_t head = mem_->imu_head_seq.load(std::memory_order_acquire);
        if(head==last_seq) return; // nothing new
        if(head - last_seq > CAPACITY){ last_seq = head - (uint32_t)CAPACITY; }
        for(uint32_t seq = last_seq+1; seq <= head; ++seq){ size_t idx = seq % CAPACITY; uint32_t s = mem_->imu_slots[idx].seq; if(s==seq){ on_sample(mem_->imu_slots[idx].data); } }
        last_seq = head;
    }
    bool try_get_pwm(PWMData& out){
        if(!mem_) return false;
        if(!mem_->pwm_ready.load(std::memory_order_acquire)) return false;
        out = mem_->pwm;
        mem_->pwm_ready.store(false,std::memory_order_release);
        // server-side telemetry (pwm receive rate)
        pwm_cnt_.fetch_add(1, std::memory_order_relaxed);
        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - pwm_start_).count();
        if(ms >= 1000){
            double secs = ms / 1000.0;
            pwm_hz_.store(double(pwm_cnt_.exchange(0)) / secs, std::memory_order_relaxed);
            pwm_start_ = now;
        }
        return true;
    }

    // Client-side convenience API (match old samples)
    void setup_rates(int,int,int){}
    void set_client_callbacks(const std::function<void(const ImuData&)>& imu_cb,
                              const std::function<void(const BaroData&)>& baro_cb){ std::lock_guard<std::mutex> lk(cb_m_); imu_cb_ = imu_cb; baro_cb_ = baro_cb; }
    void start_client_workers(){ if(running_.exchange(true)) return; timeBeginPeriod(1); imu_thread_ = std::thread([this]{ imu_loop(); }); baro_thread_ = std::thread([this]{ baro_loop(); }); }
    void stop_client_workers(){ if(!running_.exchange(false)) return; if(imu_thread_.joinable()) imu_thread_.join(); if(baro_thread_.joinable()) baro_thread_.join(); timeEndPeriod(1); }
    static PWMData make_fixed_pwm(long long ts,int seq){ PWMData p{}; p.rotor1=p.rotor2=p.rotor3=p.rotor4=1000; p.timestamp=ts; p.frequency=seq; p.is_valid=true; std::memset(p.padding,0,sizeof p.padding); return p; }
    static BaroData make_fixed_baro(long long ts,int seq){ BaroData b{}; b.altitude=10.0; b.timestamp=ts; b.frequency=seq; b.is_valid=true; std::memset(b.padding,0,sizeof b.padding); return b; }

    // Telemetry (optional)
    double get_imu_tx_hz() const { return imu_hz_.load(std::memory_order_relaxed); }
    double get_pwm_rx_hz() const { return pwm_hz_.load(std::memory_order_relaxed); }
    double get_baro_tx_hz() const { return baro_hz_.load(std::memory_order_relaxed); }
    void start_server_workers(){ auto now=std::chrono::steady_clock::now(); imu_start_=pwm_start_=baro_start_=now; imu_cnt_=pwm_cnt_=baro_cnt_=0; }

private:
    Shared* mem_; HANDLE handle_;
    std::atomic<bool> running_{false};
    std::thread imu_thread_, baro_thread_;
    std::mutex cb_m_; std::function<void(const ImuData&)> imu_cb_; std::function<void(const BaroData&)> baro_cb_;
    uint32_t last_seq_{0};
    // telemetry counters (client/server can update these if needed)
    std::atomic<double> imu_hz_{0.0}, pwm_hz_{0.0}, baro_hz_{0.0};
    std::chrono::steady_clock::time_point imu_start_{}, pwm_start_{}, baro_start_{}; std::atomic<uint64_t> imu_cnt_{0}, pwm_cnt_{0}, baro_cnt_{0};

    void imu_loop(){
        while(running_.load()){ consume_new_imu(last_seq_,[this](const ImuData& d){ std::function<void(const ImuData&)> cb; { std::lock_guard<std::mutex> lk(cb_m_); cb = imu_cb_; } if(cb) cb(d); imu_cnt_.fetch_add(1,std::memory_order_relaxed); auto now=std::chrono::steady_clock::now(); auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(now-imu_start_).count(); if(ms>=1000){ imu_hz_.store(double(imu_cnt_.exchange(0))/ (ms/1000.0),std::memory_order_relaxed); imu_start_=now; } }); std::this_thread::yield(); }
    }
    void baro_loop(){ while(running_.load()){ if(mem_){ int cur = mem_->baro_sequence.load(std::memory_order_acquire); static int last=-1; if(cur!=last && mem_->baro_ready.load(std::memory_order_acquire)){ BaroData b = mem_->baro; last = cur; std::function<void(const BaroData&)> cb; { std::lock_guard<std::mutex> lk(cb_m_); cb = baro_cb_; } if(cb) cb(b); baro_cnt_.fetch_add(1,std::memory_order_relaxed); auto now=std::chrono::steady_clock::now(); auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(now-baro_start_).count(); if(ms>=1000){ baro_hz_.store(double(baro_cnt_.exchange(0))/ (ms/1000.0),std::memory_order_relaxed); baro_start_=now; } } } std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
    }
};


