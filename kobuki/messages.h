#pragma once

#include <bitset>
#include <cstdint>

namespace kobuki {

struct BasicSideData {
    bool bumped;
    bool wheel_dropped;
    bool cliff_sensed;
    bool overcurrent;
};

struct BasicData {
    uint16_t timestamp_ms;
    BasicSideData left_data;
    BasicSideData right_data;
    BasicSideData center_data;
    uint16_t encoder_left;
    uint16_t encoder_right;
    uint8_t pwm_left;
    uint8_t pwm_right;
    std::bitset<3> buttons;
    bool is_charged;
    bool is_charging;
    bool is_using_adapter;
    bool is_docked;
    double battery_voltage;
};

struct DockingSignal {
    bool near_left;
    bool near_right;
    bool near_center;
};

struct DockingIR {
    DockingSignal left;
    DockingSignal right;
    DockingSignal center;
};

struct InertialData {
    int16_t angle;
    uint16_t angle_rate;
};

struct CliffData {
    double voltage_left;
    double voltage_right;
    double voltage_center;
};

struct Current {
    double current_left;
    double current_right;
};

struct GyroData {
    double wx;
    double wy;
    double wz;
};

struct GeneralPurposeInput {
    std::bitset<4> digital_inputs;
    double voltage_0;
    double voltage_1;
    double voltage_2;
    double voltage_3;
};

struct UDID {
    uint32_t id_0;
    uint32_t id_1;
    uint32_t id_2;
};

struct PID {
    double P;
    double I;
    double D;
};

} // namespace kobuki
