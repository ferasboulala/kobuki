#pragma once

#include <bitset>
#include <cstdint>

namespace kobuki
{

enum class SoundSequence : uint8_t
{
    On = 0,
    Off,
    Recharge,
    Button,
    Error,
    CleaningStart,
    CleaningStop,
};

struct BasicSideData
{
    bool bumped;
    bool cliff_sensed;
    // The following are no-op for center data
    bool wheel_dropped;
    bool overcurrent;
    uint16_t encoder;
    uint8_t pwm;
};

struct BasicData
{
    uint16_t timestamp_ms;
    BasicSideData left_data;
    BasicSideData right_data;
    BasicSideData center_data;
    std::bitset<3> buttons;
    bool is_charged;
    bool is_charging;
    bool is_using_adapter;
    bool is_docked;
    double battery_voltage;
};

struct DockingSignal
{
    bool near_left;
    bool near_right;
    bool near_center;
};

struct DockingIR
{
    DockingSignal left;
    DockingSignal right;
    DockingSignal center;
};

struct InertialData
{
    int16_t angle;
    uint16_t angle_rate;
};

struct CliffData
{
    double voltage_left;
    double voltage_right;
    double voltage_center;
};

struct Current
{
    double current_left;
    double current_right;
};

struct GyroData
{
    double wx;
    double wy;
    double wz;
};

struct GeneralPurposeInput
{
    std::bitset<4> digital_inputs;
    double voltage_0;
    double voltage_1;
    double voltage_2;
    double voltage_3;
};

struct UDID
{
    uint32_t id_0;
    uint32_t id_1;
    uint32_t id_2;
};

struct PID
{
    double P;
    double I;
    double D;
};

} // namespace kobuki
