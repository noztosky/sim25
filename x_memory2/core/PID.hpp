#pragma once
#include <cmath>

class PID {
public:
    PID() = default;

    void set_gains(float kp, float ki, float kd, float kff) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        kff_ = kff;
    }

    void set_limits(float imax, float out_min, float out_max) {
        imax_ = imax;
        out_min_ = out_min;
        out_max_ = out_max;
    }

    void set_d_filter_hz(float filter_hz) {
        d_filter_hz_ = filter_hz;
    }

    void reset() {
        integrator_ = 0.0f;
        last_input_ = 0.0f;
        last_derivative_ = 0.0f;
        is_first_run_ = true;
    }

    float compute(float target, float input, float dt) {
        if (dt <= 0.0f) return 0.0f;

        float error = target - input;
        
        // P-term
        float p_out = kp_ * error;

        // I-term (with anti-windup clamping)
        if (ki_ > 0.0f) {
            integrator_ += ki_ * error * dt;
            if (integrator_ > imax_) integrator_ = imax_;
            else if (integrator_ < -imax_) integrator_ = -imax_;
        }

        // D-term (Derivative-on-Measurement)
        float d_out = 0.0f;
        if (kd_ > 0.0f) {
            float raw_derivative = 0.0f;
            if (!is_first_run_) {
                // Negative because error = target - input. Derivative of error w.r.t input is negative.
                raw_derivative = -(input - last_input_) / dt;
            }

            // Apply Low-Pass Filter (LPF) to D-term input
            float filtered_derivative = raw_derivative;
            if (d_filter_hz_ > 0.0f && !is_first_run_) {
                float rc = 1.0f / (2.0f * 3.14159265f * d_filter_hz_);
                float alpha = dt / (rc + dt);
                filtered_derivative = last_derivative_ + alpha * (raw_derivative - last_derivative_);
            }

            d_out = kd_ * filtered_derivative;
            last_derivative_ = filtered_derivative;
        }

        // Feedforward term
        float ff_out = kff_ * target;

        // Combined output
        float output = p_out + integrator_ + d_out + ff_out;

        // Output clamping
        if (output > out_max_) output = out_max_;
        else if (output < out_min_) output = out_min_;

        last_input_ = input;
        is_first_run_ = false;

        return output;
    }

    float get_integrator() const { return integrator_; }

private:
    float kp_{0.0f};
    float ki_{0.0f};
    float kd_{0.0f};
    float kff_{0.0f};

    float imax_{0.0f};
    float out_min_{-1.0f};
    float out_max_{1.0f};

    float d_filter_hz_{0.0f};

    float integrator_{0.0f};
    float last_input_{0.0f};
    float last_derivative_{0.0f};
    bool is_first_run_{true};
};
