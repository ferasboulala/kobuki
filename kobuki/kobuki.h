#pragma once
#include "messages.h"

namespace kobuki {

struct Feedback {
    uint16_t timestamp_ms;
    bool bumped_left;
    bool bumped_right;
    bool bumped_center;
    bool wheel_dropped_left;
    bool wheel_dropped_right;
    bool cliff_detected_left;
    bool cliff_detected_right;
    uint16_t left_ticks;
    uint16_t right_ticks;
    int8_t left_pwm;
    int8_t right_pwm;
    bool pressed_button_0;
    bool pressed_button_1;
    bool pressed_button_2;
    Charger charger;
    double battery_voltage;
    bool overcurrent_left;
    bool overcurrent_right;
    bool 
};

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
