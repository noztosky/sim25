#pragma once

#include <string>
#include <functional>
#include "D:\open\airsim\x_memory\x_xsim.h"

class XSimIo {
public:
	bool connect(const char* name)
	{
		return xs_.client_connect(name);
	}
	const x_xsim& handle() const { return xs_; }
	x_xsim& handle() { return xs_; }

	// Thin wrappers for convenience
	template<typename Fn>
	void consumeTelem(uint32_t& last_seq, Fn&& on_sample) { xs_.consume_telem(last_seq, std::forward<Fn>(on_sample)); }
	void submitPwm(const XSimPwm& p) { xs_.submit_pwm(p); }
	double getPwmTxHz() const { return xs_.get_pwm_tx_hz(); }
 
private:
	x_xsim xs_{};
};


