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
#include <chrono>

// Composite telemetry for FC experiments
// Units:
// - acc (m/s^2) body frame
// - gyro (rad/s) body frame
// - quat (w,x,y,z) float
// - loc_ned (m) world NED
// - alt (m)
// - mag (uT or normalized), body frame
// - timestamp ns, seq increasing
struct XSimTelemetry {
    double acc[3];
    double gyro[3];
    float quat[4];
    double loc_ned[3];
    double alt;
    double mag[3];
    long long timestamp;
    int seq;
    bool is_valid;
    char padding[3];
};

struct XSimPwm {
    int rotor1;
    int rotor2;
    int rotor3;
    int rotor4;
    long long timestamp;
    int seq;
    bool is_valid;
    char padding[3];
};

template <size_t CAP>
struct XSimRingData {
    struct SlotT { XSimTelemetry data; uint32_t seq; };
    struct SlotP { XSimPwm data; uint32_t seq; };
    SlotT telem_slots[CAP];
    std::atomic<uint32_t> telem_head_seq;
    SlotP pwm_slots[CAP];
    std::atomic<uint32_t> pwm_head_seq;
};

class x_xsim {
public:
    static constexpr size_t CAPACITY = 32;
    using Shared = XSimRingData<CAPACITY>;

    x_xsim(): mem_(nullptr), handle_(nullptr) {}
    ~x_xsim(){ close(); }

    bool server_create(const char* name){
        close();
        handle_ = ::CreateFileMappingA(INVALID_HANDLE_VALUE,nullptr,PAGE_READWRITE,0,(DWORD)sizeof(Shared),name);
        if(!handle_) return false;
        mem_ = static_cast<Shared*>(::MapViewOfFile(handle_,FILE_MAP_ALL_ACCESS,0,0,sizeof(Shared)));
        if(!mem_){::CloseHandle(handle_); handle_=nullptr; return false;}
        std::memset(mem_,0,sizeof(Shared));
        mem_->telem_head_seq.store(0,std::memory_order_relaxed);
        mem_->pwm_head_seq.store(0,std::memory_order_relaxed);
        reset_counters();
        return true;
    }
    bool client_connect(const char* name){
        close();
        handle_ = ::OpenFileMappingA(FILE_MAP_ALL_ACCESS,FALSE,name);
        if(!handle_) return false;
        mem_ = static_cast<Shared*>(::MapViewOfFile(handle_,FILE_MAP_ALL_ACCESS,0,0,sizeof(Shared)));
        if(!mem_){::CloseHandle(handle_); handle_=nullptr; return false;}
        reset_counters();
        return true;
    }
    void close(){ if(mem_){::UnmapViewOfFile(mem_); mem_=nullptr;} if(handle_){::CloseHandle(handle_); handle_=nullptr;} }

    // Server-side publish
    void publish_telem(const XSimTelemetry& d){ if(!mem_) return; uint32_t next = mem_->telem_head_seq.load(std::memory_order_relaxed)+1; size_t idx = next % CAPACITY; mem_->telem_slots[idx].data = d; mem_->telem_slots[idx].seq = next; mem_->telem_head_seq.store(next,std::memory_order_release); telem_cnt_.fetch_add(1,std::memory_order_relaxed); tick(telem_start_, telem_cnt_, telem_hz_); }
    // Client-side submit PWM
    void submit_pwm(const XSimPwm& p){ if(!mem_) return; uint32_t next = mem_->pwm_head_seq.load(std::memory_order_relaxed)+1; size_t idx = next % CAPACITY; mem_->pwm_slots[idx].data = p; mem_->pwm_slots[idx].seq = next; mem_->pwm_head_seq.store(next,std::memory_order_release); pwm_tx_cnt_.fetch_add(1,std::memory_order_relaxed); tick(pwm_tx_start_, pwm_tx_cnt_, pwm_tx_hz_); }

    // Client-side consume telemetry (actual dequeued count)
    template<typename Fn>
    void consume_telem(uint32_t& last_seq, Fn&& on_sample){ if(!mem_) return; uint32_t head = mem_->telem_head_seq.load(std::memory_order_acquire); if(head==last_seq) return; if(head - last_seq > CAPACITY){ last_seq = head - (uint32_t)CAPACITY; } for(uint32_t s = last_seq+1; s<=head; ++s){ size_t idx = s % CAPACITY; if(mem_->telem_slots[idx].seq==s){ on_sample(mem_->telem_slots[idx].data); telem_rx_cnt_.fetch_add(1,std::memory_order_relaxed); tick(telem_rx_start_, telem_rx_cnt_, telem_rx_hz_); } } last_seq = head; }
    // Server-side consume PWM
    bool try_get_pwm(uint32_t& last_seq, XSimPwm& out){ if(!mem_) return false; uint32_t head = mem_->pwm_head_seq.load(std::memory_order_acquire); if(head==last_seq) return false; if(head - last_seq > CAPACITY){ last_seq = head - (uint32_t)CAPACITY; } uint32_t next = last_seq+1; size_t idx = next % CAPACITY; if(mem_->pwm_slots[idx].seq != next) return false; out = mem_->pwm_slots[idx].data; last_seq = next; pwm_rx_cnt_.fetch_add(1,std::memory_order_relaxed); tick(pwm_rx_start_, pwm_rx_cnt_, pwm_rx_hz_); return true; }

    // Telemetry
    double get_telem_tx_hz() const { return telem_hz_.load(std::memory_order_relaxed); }
    double get_telem_rx_hz() const { return telem_rx_hz_.load(std::memory_order_relaxed); }
    double get_pwm_tx_hz() const { return pwm_tx_hz_.load(std::memory_order_relaxed); }
    double get_pwm_rx_hz() const { return pwm_rx_hz_.load(std::memory_order_relaxed); }

private:
    Shared* mem_;
    HANDLE handle_;
    std::atomic<double> telem_hz_{0.0}, telem_rx_hz_{0.0}, pwm_tx_hz_{0.0}, pwm_rx_hz_{0.0};
    std::atomic<uint64_t> telem_cnt_{0}, telem_rx_cnt_{0}, pwm_tx_cnt_{0}, pwm_rx_cnt_{0};
    std::chrono::steady_clock::time_point telem_start_{}, telem_rx_start_{}, pwm_tx_start_{}, pwm_rx_start_{};

    static void tick(std::chrono::steady_clock::time_point& tp, std::atomic<uint64_t>& cnt, std::atomic<double>& hz){ auto now = std::chrono::steady_clock::now(); if(tp.time_since_epoch().count()==0) { tp = now; cnt.store(0,std::memory_order_relaxed); return; } auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - tp).count(); if(ms>=1000){ double secs = ms/1000.0; hz.store(double(cnt.exchange(0))/secs, std::memory_order_relaxed); tp = now; } }
    void reset_counters(){ auto now = std::chrono::steady_clock::now(); telem_start_=telem_rx_start_=pwm_tx_start_=pwm_rx_start_=now; telem_cnt_=0; telem_rx_cnt_=0; pwm_tx_cnt_=0; pwm_rx_cnt_=0; }
};


