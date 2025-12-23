#pragma once

#include <cstdint>
#include <chrono>
#include <cstring>
#include "x_xsim.h"

class RotorWriter {
public:
	explicit RotorWriter(x_xsim& xs) : xs_(xs) {}

	void write(uint16_t r1, uint16_t r2, uint16_t r3, uint16_t r4)
	{
		auto now = std::chrono::high_resolution_clock::now();
		XSimPwm p{};
		p.rotor1 = static_cast<int>(r1);
		p.rotor2 = static_cast<int>(r2);
		p.rotor3 = static_cast<int>(r3);
		p.rotor4 = static_cast<int>(r4);
		p.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
		p.seq = ++seq_;
		p.is_valid = true;
		std::memset(p.padding, 0, sizeof p.padding);
		xs_.submit_pwm(p);
	}

private:
	x_xsim& xs_;
	int seq_ = 0;
};


