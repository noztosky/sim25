#pragma once

// Generic Driver Interface
class Driver_IF {
public:
    virtual ~Driver_IF() = default;

    virtual bool connect() = 0;
    virtual void close() = 0;
    virtual bool is_connected() const = 0;
};
