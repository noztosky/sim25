#ifndef ACT_IF_HPP
#define ACT_IF_HPP

#include <vector>
#include <cstdint>

struct PWMData {
    uint64_t timestamp_ns;
    std::vector<float> pwm_values; // 0.0 to 1.0
};

class ACT_IF {
public:
    virtual ~ACT_IF() = default;
    virtual bool init() = 0;
    virtual bool write(const PWMData& data) = 0;
};

#endif
