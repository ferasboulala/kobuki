#pragma once
#include "messages.h"

namespace kobuki {

class Kobuki {
public:
    Kobuki(const std::string &device = "/dev/kobuki");
    ~Kobuki() = default;

    static double tick_to_meters(uint16_t ticks);

    bool bumped_left() const;
    bool bumped_right() const;
    bool bumped_center() const;

private:
};

} // namespace kobuki
