#pragma once

class PidController {
public:
	PidController() = default;
	void configure(float kp, float ki, float integral_limit_us)
	{
		kp_ = kp; ki_ = ki; i_limit_ = integral_limit_us;
	}
	void reset()
	{
		i_us_ = 0.0f;
	}
	float compute(float error_rad, double dt_sec, bool allow_integral)
	{
		float p = kp_ * (-error_rad);
		if (allow_integral && ki_ > 0.0f && dt_sec > 0.0) {
			i_us_ += ki_ * (-error_rad) * static_cast<float>(dt_sec);
			if (i_us_ > i_limit_) i_us_ = i_limit_;
			if (i_us_ < -i_limit_) i_us_ = -i_limit_;
		}
		return p + i_us_;
	}
	float getIntegralUs() const { return i_us_; }
private:
	float kp_{0.0f};
	float ki_{0.0f};
	float i_limit_{0.0f};
	float i_us_{0.0f};
};


