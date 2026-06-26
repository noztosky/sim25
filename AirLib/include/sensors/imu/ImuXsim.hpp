#ifndef msr_airlib_ImuXsim_hpp
#define msr_airlib_ImuXsim_hpp

#include "common/Common.hpp"
#include "ImuBase.hpp"
#include "common/VectorMath.hpp"
#include "common/XlabUeMetrics.hpp"
#include "common/EarthUtils.hpp"
// use new x_xsim shared memory for composite telemetry
#include "../../../../x_memory/shm/x_xsim.h"
#include "common/XlabXMemoryAdapter.hpp"
#include <fstream>
#include <chrono>
#include <cstring>

namespace msr
{
namespace airlib
{

class ImuXsim : public ImuBase
{
public:
    ImuXsim(const AirSimSettings::ImuSetting& setting = AirSimSettings::ImuSetting())
        : ImuBase(setting.sensor_name)
    {
        unused(setting);
    }

    virtual void resetImplementation() override
    {
        last_time_ = clock()->nowNanos();

        last_log_time_ = clock()->nowNanos();
        update_count_ = 0;

        const GroundTruth& ground_truth = getGroundTruth();
        real_T pitch_r = 0, roll_r = 0, yaw_r = 0;
        VectorMath::toEulerianAngle(ground_truth.kinematics->pose.orientation, pitch_r, roll_r, yaw_r);
        last_yaw_rad_ = yaw_r;
        yaw_changed_count_ = 0;

        updateOutput();

        if (!xmem_inited_) {
            // Synchronize SHM output period with PhysicsLoopPeriod (default 8kHz/125us)
            long long phys_period = Settings::singleton().getInt("PhysicsLoopPeriod", 125000);
            target_hz_ = 1e9f / phys_period;
            period_ns_ = phys_period;

            next_write_tp_ns_ = clock()->nowNanos() + period_ns_;
            if (!xsim_) xsim_.reset(new x_xsim());
            xsim_->server_create("AirSimXsim");
            xmem_inited_ = true;
        }

        // Initialize BARO/MAG 100 Hz throttling schedule and caches
        baro_target_hz_ = 100.0f;
        mag_target_hz_  = 100.0f;
        baro_period_ns_ = static_cast<TTimePoint>(1e9 / static_cast<double>(baro_target_hz_));
        mag_period_ns_  = static_cast<TTimePoint>(1e9 / static_cast<double>(mag_target_hz_));
        const TTimePoint now_ns = clock()->nowNanos();
        next_baro_update_ns_ = now_ns + baro_period_ns_;
        next_mag_update_ns_  = now_ns + mag_period_ns_;
        // prime caches
        const auto& p0 = ground_truth.kinematics->pose.position;
        last_alt_ = -static_cast<double>(p0.z());
        {
            Vector3r mag_world0 = EarthUtils::getMagField(ground_truth.environment->getState().geo_point) * 1E4f; // Tesla->Gauss
            Vector3r mag_body0 = VectorMath::transformToBodyFrame(mag_world0, ground_truth.kinematics->pose.orientation, true);
            last_mag_[0] = static_cast<double>(mag_body0.x());
            last_mag_[1] = static_cast<double>(mag_body0.y());
            last_mag_[2] = static_cast<double>(mag_body0.z());
        }
        // Initialize LOC 10 Hz throttling
        loc_target_hz_ = 10.0f;
        loc_period_ns_ = static_cast<TTimePoint>(1e9 / static_cast<double>(loc_target_hz_));
        next_loc_update_ns_ = now_ns + loc_period_ns_;
        last_loc_[0] = static_cast<double>(p0.x());
        last_loc_[1] = static_cast<double>(p0.y());
        last_loc_[2] = static_cast<double>(p0.z());
        last_vel_[0] = 0.0;
        last_vel_[1] = 0.0;
        last_vel_[2] = 0.0;
    }

    virtual void update() override
    {
        // Measure REAL-TIME interval between physics-loop sensor updates (= SHM
        // sampling cadence). This does NOT change the scheduler; it only observes it.
        measureLoopJitter();

        ImuBase::update();

        /* yaw processing skipped for performance */

        updateOutput();

        // publish to x_xsim at target_hz_ (synchronized with physics by default)
        if (xmem_inited_ && target_hz_ > 0) {
            const TTimePoint now_ns = clock()->nowNanos();
            if (static_cast<long long>(now_ns - next_write_tp_ns_) >= 0) {
                const auto& out = getOutput();
                const GroundTruth& gt = getGroundTruth();
                const auto& q = gt.kinematics->pose.orientation;
                const auto& p = gt.kinematics->pose.position;

                // update slow channels on schedule using current now_ns
                if (static_cast<long long>(now_ns - next_loc_update_ns_) >= 0) {
                    last_loc_[0] = static_cast<double>(p.x());
                    last_loc_[1] = static_cast<double>(p.y());
                    last_loc_[2] = static_cast<double>(p.z());
                    last_vel_[0] = static_cast<double>(gt.kinematics->twist.linear.x());
                    last_vel_[1] = static_cast<double>(gt.kinematics->twist.linear.y());
                    last_vel_[2] = static_cast<double>(gt.kinematics->twist.linear.z());
                    next_loc_update_ns_ += loc_period_ns_;
                    ++loc_tick_cnt_;
                }
                if (static_cast<long long>(now_ns - next_baro_update_ns_) >= 0) {
                    last_alt_ = -static_cast<double>(p.z());
                    last_pressure_ = 101325.0 * std::pow(1.0 - 2.25577e-5 * last_alt_, 5.25588);
                    last_temp_ = 15.0 - 0.0065 * last_alt_;
                    next_baro_update_ns_ += baro_period_ns_;
                    ++baro_tick_cnt_;
                }
                if (static_cast<long long>(now_ns - next_mag_update_ns_) >= 0) {
                    Vector3r mag_world = EarthUtils::getMagField(gt.environment->getState().geo_point) * 1E4f; // Tesla->Gauss
                    Vector3r mag_body = VectorMath::transformToBodyFrame(mag_world, gt.kinematics->pose.orientation, true);
                    last_mag_[0] = static_cast<double>(mag_body.x());
                    last_mag_[1] = static_cast<double>(mag_body.y());
                    last_mag_[2] = static_cast<double>(mag_body.z());
                    next_mag_update_ns_ += mag_period_ns_;
                    ++mag_tick_cnt_;
                }

                // emit one or more samples to catch up (capped)
                int batch_emits = 0;
                constexpr int kMaxBatch = 500; // Increased significantly for high RTF stability
                while (static_cast<long long>(now_ns - next_write_tp_ns_) >= 0 && batch_emits < kMaxBatch) {
                    XSimTelemetry d; // fields set below
                    d.gyro[0] = static_cast<double>(gt.kinematics->twist.angular.x());
                    d.gyro[1] = static_cast<double>(gt.kinematics->twist.angular.y());
                    d.gyro[2] = static_cast<double>(gt.kinematics->twist.angular.z());
                    d.acc[0] = static_cast<double>(out.linear_acceleration.x());
                    d.acc[1] = static_cast<double>(out.linear_acceleration.y());
                    d.acc[2] = static_cast<double>(out.linear_acceleration.z());
                    d.quat[0] = q.w(); d.quat[1] = q.x(); d.quat[2] = q.y(); d.quat[3] = q.z();
                    d.loc_ned[0] = last_loc_[0];
                    d.loc_ned[1] = last_loc_[1];
                    d.loc_ned[2] = last_loc_[2];
                    d.vel_ned[0] = last_vel_[0];
                    d.vel_ned[1] = last_vel_[1];
                    d.vel_ned[2] = last_vel_[2];
                    d.alt = last_alt_;
                    d.pressure = last_pressure_;
                    d.temperature = last_temp_;
                    d.mag[0] = last_mag_[0];
                    d.mag[1] = last_mag_[1];
                    d.mag[2] = last_mag_[2];
                    d.timestamp = static_cast<long long>(next_write_tp_ns_);
                    d.seq = static_cast<int>(++imu_seq_);
                    d.is_valid = true;
                    if (xsim_) xsim_->publish_telem(d);
                    next_write_tp_ns_ += period_ns_;
                    ++batch_emits;
                    ++emit_tick_cnt_;
                }
                // if still behind (un-emitted scheduled samples remain), count them as skipped and resync
                if (static_cast<long long>(now_ns - next_write_tp_ns_) >= 0) {
                    // number of scheduled periods overdue including the current one
                    long long overdue = static_cast<long long>((now_ns - next_write_tp_ns_) / period_ns_) + 1;
                    if (overdue > 0) skip_tick_cnt_ += static_cast<uint32_t>(overdue);
                    next_write_tp_ns_ = now_ns + period_ns_;
                }
            }
        }

        ++update_count_;
        const TTimeDelta elapsed_sec = clock()->elapsedSince(last_log_time_);
        if (elapsed_sec >= 1.0) {
            const double hz = static_cast<double>(update_count_) / static_cast<double>(elapsed_sec);
            XlabUeMetrics::setImuHz(static_cast<int>(hz + 0.5));
            const double baro_hz = static_cast<double>(baro_tick_cnt_) / static_cast<double>(elapsed_sec);
            XlabUeMetrics::setBaroHz(static_cast<int>(baro_hz + 0.5));
            const double mag_hz = static_cast<double>(mag_tick_cnt_) / static_cast<double>(elapsed_sec);
            XlabUeMetrics::setMagHz(static_cast<int>(mag_hz + 0.5));
            const double loc_hz = static_cast<double>(loc_tick_cnt_) / static_cast<double>(elapsed_sec);
            XlabUeMetrics::setLocHz(static_cast<int>(loc_hz + 0.5));
            const double emit_hz = static_cast<double>(emit_tick_cnt_) / static_cast<double>(elapsed_sec);
            XlabUeMetrics::setImuEmitHz(static_cast<int>(emit_hz + 0.5));
            const double skip_hz = static_cast<double>(skip_tick_cnt_) / static_cast<double>(elapsed_sec);
            XlabUeMetrics::setImuSkipHz(static_cast<int>(skip_hz + 0.5));
            // keep counters but suppress per-line IMU log to avoid duplicate lines; PWM logger prints combined line
            last_log_time_ = clock()->nowNanos();
            update_count_ = 0;
            baro_tick_cnt_ = 0; mag_tick_cnt_ = 0; loc_tick_cnt_ = 0; emit_tick_cnt_ = 0; skip_tick_cnt_ = 0;
            yaw_changed_count_ = 0;
        }
    }

    virtual ~ImuXsim() = default;

private:
    void updateOutput()
    {
        Output output;
        const GroundTruth& ground_truth = getGroundTruth();

        output.angular_velocity = ground_truth.kinematics->twist.angular;
        output.linear_acceleration = ground_truth.kinematics->accelerations.linear - ground_truth.environment->getState().gravity;
        output.orientation = ground_truth.kinematics->pose.orientation;

        output.linear_acceleration = VectorMath::transformToBodyFrame(
            output.linear_acceleration,
            ground_truth.kinematics->pose.orientation,
            true);

        output.time_stamp = clock()->nowNanos();
        setOutput(output);
    }

    // Observe the real wall-clock interval between successive sensor updates,
    // i.e. the actual cadence at which telemetry is published to SHM. Aggregates
    // per real second and surfaces to HUD (XlabUeMetrics) + log + CSV.
    void measureLoopJitter()
    {
        using sclock = std::chrono::steady_clock;
        const auto now_tp = sclock::now();
        if (jit_first_) {
            jit_first_ = false;
            jit_last_tp_ = now_tp;
            jit_report_tp_ = now_tp;
            return;
        }
        const double dt_us = std::chrono::duration<double, std::micro>(now_tp - jit_last_tp_).count();
        jit_last_tp_ = now_tp;

        const double target_us = (period_ns_ > 0) ? static_cast<double>(period_ns_) / 1000.0 : 0.0;
        double dev = dt_us - target_us;
        if (dev < 0.0) dev = -dev;

        ++jit_count_;
        jit_dt_sum_us_ += dt_us;
        if (dt_us > jit_dt_max_us_) jit_dt_max_us_ = dt_us;
        if (dt_us < jit_dt_min_us_) jit_dt_min_us_ = dt_us;
        jit_dev_sum_us_ += dev;
        if (dev > jit_dev_max_us_) jit_dev_max_us_ = dev;
        int b = static_cast<int>(dev + 0.5);
        if (b < 0) b = 0;
        if (b > JIT_BUCKETS) b = JIT_BUCKETS;
        ++jit_hist_[b];

        const double since_report = std::chrono::duration<double>(now_tp - jit_report_tp_).count();
        if (since_report >= 1.0 && jit_count_ > 0) {
            const double mean_dt = jit_dt_sum_us_ / static_cast<double>(jit_count_);
            const double mean_dev = jit_dev_sum_us_ / static_cast<double>(jit_count_);
            const double achieved_hz = (mean_dt > 0.0) ? 1e6 / mean_dt : 0.0;
            // p95/p99 of deviation from the 1us-bucket histogram
            const uint64_t t95 = static_cast<uint64_t>(0.95 * static_cast<double>(jit_count_));
            const uint64_t t99 = static_cast<uint64_t>(0.99 * static_cast<double>(jit_count_));
            uint64_t cum = 0; int p95 = JIT_BUCKETS, p99 = JIT_BUCKETS; bool g95 = false, g99 = false;
            for (int i = 0; i <= JIT_BUCKETS; ++i) {
                cum += jit_hist_[i];
                if (!g95 && cum >= t95) { p95 = i; g95 = true; }
                if (!g99 && cum >= t99) { p99 = i; g99 = true; break; }
            }
            XlabUeMetrics::setLoopHz(static_cast<int>(achieved_hz + 0.5));
            XlabUeMetrics::setLoopJitMeanUs(static_cast<int>(mean_dev + 0.5));
            XlabUeMetrics::setLoopJitP99Us(p99);
            XlabUeMetrics::setLoopJitMaxUs(static_cast<int>(jit_dev_max_us_ + 0.5));
            Utils::log(Utils::stringf(
                "[XsimJit] target=%.0fHz achieved=%.1fHz | dt(us) mean=%.2f min=%.2f max=%.2f | jitter(us) mean=%.2f p95=%d p99=%d max=%.2f | n=%llu",
                target_hz_, achieved_hz, mean_dt, jit_dt_min_us_, jit_dt_max_us_,
                mean_dev, p95, p99, jit_dev_max_us_, static_cast<unsigned long long>(jit_count_)));
            writeJitterCsv(achieved_hz, mean_dt, jit_dt_min_us_, jit_dt_max_us_, mean_dev, p95, p99, jit_dev_max_us_);
            // reset window
            jit_report_tp_ = now_tp;
            jit_count_ = 0;
            jit_dt_sum_us_ = 0.0; jit_dt_max_us_ = 0.0; jit_dt_min_us_ = 1e18;
            jit_dev_sum_us_ = 0.0; jit_dev_max_us_ = 0.0;
            std::memset(jit_hist_, 0, sizeof(jit_hist_));
        }
    }

    void writeJitterCsv(double achieved_hz, double dt_mean, double dt_min, double dt_max,
                        double jit_mean, int jit_p95, int jit_p99, double jit_max)
    {
        if (!jit_csv_open_) {
            jit_csv_.open("xsim_loop_jitter.csv", std::ios::out | std::ios::trunc);
            if (jit_csv_.is_open())
                jit_csv_ << "target_hz,achieved_hz,dt_mean_us,dt_min_us,dt_max_us,"
                            "jit_mean_us,jit_p95_us,jit_p99_us,jit_max_us\n";
            jit_csv_open_ = true;
        }
        if (jit_csv_.is_open()) {
            jit_csv_ << static_cast<long long>(target_hz_ + 0.5) << "," << achieved_hz << ","
                     << dt_mean << "," << dt_min << "," << dt_max << ","
                     << jit_mean << "," << jit_p95 << "," << jit_p99 << "," << jit_max << "\n";
            jit_csv_.flush();
        }
    }

private:
    TTimePoint last_time_ = 0;
    TTimePoint last_log_time_ = 0;
    uint32_t update_count_ = 0;
    uint32_t yaw_changed_count_ = 0;
    real_T last_yaw_rad_ = 0;

    std::unique_ptr<x_xsim> xsim_;
    bool xmem_inited_ = false;
    float target_hz_ = 8000.0f;
    // scheduling for consistent rate
    TTimePoint next_write_tp_ns_ = 0;
    TTimePoint period_ns_ = static_cast<TTimePoint>(1e9 / 8000.0);
    uint32_t imu_seq_ = 0;

    // 100 Hz throttling for BARO and MAG in SHM telemetry
    float baro_target_hz_ = 100.0f;
    float mag_target_hz_  = 100.0f;
    TTimePoint baro_period_ns_ = 0;
    TTimePoint mag_period_ns_  = 0;
    TTimePoint next_baro_update_ns_ = 0;
    TTimePoint next_mag_update_ns_  = 0;
    double last_alt_ = 0.0;
    double last_pressure_ = 101325.0;
    double last_temp_ = 15.0;
    double last_mag_[3] = {0.0, 0.0, 0.0};
    uint32_t baro_tick_cnt_ = 0;
    uint32_t mag_tick_cnt_ = 0;
    // LOC throttling state (10 Hz)
    float loc_target_hz_ = 10.0f;
    TTimePoint loc_period_ns_ = 0;
    TTimePoint next_loc_update_ns_ = 0;
    double last_loc_[3] = {0.0, 0.0, 0.0};
    double last_vel_[3] = {0.0, 0.0, 0.0};
    uint32_t loc_tick_cnt_ = 0;
    // producer metrics
    uint32_t emit_tick_cnt_ = 0;
    uint32_t skip_tick_cnt_ = 0;

    // --- real-time loop jitter measurement (SHM sampling cadence) ---
    static constexpr int JIT_BUCKETS = 4096; // deviation histogram, 1us buckets (0..4ms) + overflow
    std::chrono::steady_clock::time_point jit_last_tp_{};
    std::chrono::steady_clock::time_point jit_report_tp_{};
    bool jit_first_ = true;
    uint64_t jit_count_ = 0;
    double jit_dt_sum_us_ = 0.0;
    double jit_dt_max_us_ = 0.0;
    double jit_dt_min_us_ = 1e18;
    double jit_dev_sum_us_ = 0.0;
    double jit_dev_max_us_ = 0.0;
    uint32_t jit_hist_[JIT_BUCKETS + 1] = {0};
    std::ofstream jit_csv_;
    bool jit_csv_open_ = false;
};

}
}

#endif
