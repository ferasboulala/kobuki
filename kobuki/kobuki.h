#pragma once

#include "messages.h"
#include "protocol.h"

#include <array>
#include <string>
#include <thread>
#include <mutex>

#define MUTEX_ACCESSOR(type, label) bool label(type& label, bool block)
#define MUTEX_ASSIGN(type, label) void set_##label(const type& label)

namespace kobuki {

template <typename T>
struct EventField {
    int efd;
    std::mutex mutex;
    T field;
};

class Kobuki {
public:
    static Kobuki* create(const char* device = "/dev/kobuki");
    ~Kobuki();

    bool ok() const { return m_run; }

    MUTEX_ACCESSOR(BasicData, basic_data);
    MUTEX_ACCESSOR(DockingIR, docking_ir);
    MUTEX_ACCESSOR(InertialData, inertial_data);
    MUTEX_ACCESSOR(CliffData, cliff_data);
    MUTEX_ACCESSOR(Current, current);
    MUTEX_ACCESSOR(std::string, hardware_version);
    MUTEX_ACCESSOR(std::string, firmware_version);
    MUTEX_ACCESSOR(GyroData, gyro_data);
    MUTEX_ACCESSOR(GeneralPurposeInput, gpi);
    MUTEX_ACCESSOR(UDID, udid);
    MUTEX_ACCESSOR(PID, pid);

    // TODO : Move to another file
    static double ticks_to_meters(uint16_t ticks);
private:
    static constexpr size_t N_EFD = 8;

    Kobuki(FILE* file, const std::array<int, N_EFD> &efds);

    void read();
    bool process_packet(const char* buffer, uint8_t length);

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

    MUTEX_ASSIGN(BasicData, basic_data);
    MUTEX_ASSIGN(DockingIR, docking_ir);
    MUTEX_ASSIGN(InertialData, inertial_data);
    MUTEX_ASSIGN(CliffData, cliff_data);
    MUTEX_ASSIGN(Current, current);
    MUTEX_ASSIGN(std::string, hardware_version);
    MUTEX_ASSIGN(std::string, firmware_version);
    MUTEX_ASSIGN(GyroData, gyro_data);
    MUTEX_ASSIGN(GeneralPurposeInput, gpi);
    MUTEX_ASSIGN(UDID, udid);
    MUTEX_ASSIGN(PID, pid);

private:
    FILE* m_file;
    std::thread m_reading_thread;
    bool m_run;

    EventField<BasicData> m_basic_data;
    EventField<DockingIR> m_docking_ir;
    EventField<InertialData> m_inertial_data;
    EventField<CliffData> m_cliff_data;
    EventField<Current> m_current;
    EventField<std::string> m_hardware_version;
    EventField<std::string> m_firmware_version;
    EventField<GyroData> m_gyro_data;
    EventField<GeneralPurposeInput> m_gpi;
    EventField<UDID> m_udid;
    EventField<PID> m_pid;
};

} // namespace kobuki
