#pragma once

#include <array>
#include <mutex>
#include <string>
#include <thread>
#include <cmath>

#include "messages.h"
#include "protocol.h"

inline constexpr double DEG2RAD(double d) {
  return d / 180.0 * M_PI;
}

template <typename T>
inline int SIGNOF(T x) {
  return (x * x) / x > 0 ? 1 : -1;
}

namespace kobuki
{
// Put these in kobuki::limits
static constexpr double WHEEL_BASE = 0.230;
static constexpr double WHEEL_RADIUS = 0.035;
static constexpr double WHEEL_WIDTH = 0.021;
static constexpr double MAX_TRANS_VELOCITY = 0.7;
static constexpr double MAX_ROT_VELOCITY = DEG2RAD(180);
static constexpr double MAX_ROT_VELOCITY_SMOOTH = DEG2RAD(110);
static constexpr double MAX_PAYLOAD = 5;

template <typename T>
struct EventField
{
    int efd;
    std::mutex mutex;
    T field;
};

class Kobuki
{
public:
    static Kobuki* create(const char* device = "/dev/kobuki");
    ~Kobuki();

    bool ok() const { return m_run; }

    bool get_basic_data(BasicData& basic_data, bool block);
    bool get_docking_ir(DockingIR& docking_ir, bool block);
    bool get_inertial_data(InertialData& inertial_data, bool block);
    bool get_cliff_data(CliffData& cliff_data, bool block);
    bool get_current(Current& current, bool block);
    bool get_gyro_data(GyroData& gyro_data, bool block);
    bool get_gpi(GeneralPurposeInput& gpi, bool block);
    bool get_pid(PID& pid, bool block);

    std::string get_hardware_version() const;
    std::string get_firmware_version() const;
    UDID get_udid() const;

    void set_motion(double velocity, double radius);
    void pure_translation(double velocity) { set_motion(velocity, 0); }
    void pure_rotation(double angular_velocity) { set_motion(std::fabs(angular_velocity) * WHEEL_BASE / 2.0, SIGNOF(angular_velocity) * 0.001); };
    void set_sound(double frequency, double duration);
    void set_sound_sequence(SoundSequence sequence);
    // TODO
    void set_digital_output(bool channel_0, bool channel_2, bool channel_3, bool channel_4);
    // TODO
    void set_power_output(bool power_3_3, bool power_5, bool power_12_5, bool power_12_1_5);
    void set_leds(bool led_1_green, bool led_1_red, bool led_2_green, bool led_2_red);
    // TODO
    void set_pid(double p, double i, double d);

    // TODO : Move to another file
    static double ticks_to_meters(uint16_t ticks);

private:
    static constexpr size_t N_EFD = 8;

    Kobuki(FILE* file, const std::array<int, N_EFD>& efds);

    void spin();
    bool process_packet(const char* buffer, uint8_t length);
    bool find_packet_header();
    uint8_t checksum(uint8_t packet_length, const char* buffer);
    bool validate_checksum(uint8_t packet_length, const char* buffer);

    template <typename T>
    void send_msg(const T& msg);
    void request_identifiers();

    bool on_msg(const protocol::BasicSensorData& basic_sensor_data);
    bool on_msg(const protocol::DockingIR& docking_ir);
    bool on_msg(const protocol::InertialSensorData& inertial_sensor_data);
    bool on_msg(const protocol::CliffSensorData& cliff_sensor_data);
    bool on_msg(const protocol::Current& current);
    bool on_msg(const protocol::HardwareVersion& hardware_version);
    bool on_msg(const protocol::FirmwareVersion& firmware_version);
    bool on_msg(const protocol::RawData3AxisGyro& raw_gyro_data);
    bool on_msg(const protocol::GeneralPurposeInput& gpi);
    bool on_msg(const protocol::UniqueDeviceIdentifier& udid);
    bool on_msg(const protocol::ControllerInfo& controller_info);

    void set_basic_data(const BasicData& basic_data);
    void set_docking_ir(const DockingIR& docking_ir);
    void set_inertial_data(const InertialData& inertial_data);
    void set_cliff_data(const CliffData& cliff_data);
    void set_current(const Current& current);
    void set_gyro_data(const GyroData& gyro_data);
    void set_gpi(const GeneralPurposeInput& gpi);
    void set_pid(const PID& pid);

private:
    FILE* m_file;
    std::thread m_reading_thread;
    bool m_run;

    protocol::DigitalOutput m_cached_output;

    EventField<BasicData> m_basic_data;
    EventField<DockingIR> m_docking_ir;
    EventField<InertialData> m_inertial_data;
    EventField<CliffData> m_cliff_data;
    EventField<Current> m_current;
    EventField<GyroData> m_gyro_data;
    EventField<GeneralPurposeInput> m_gpi;
    EventField<PID> m_pid;

    std::string m_hardware_version;
    std::string m_firmware_version;
    UDID m_udid;
};

}  // namespace kobuki
