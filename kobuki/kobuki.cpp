#include "kobuki.h"

#include "messages.h"
#include "log.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <sys/eventfd.h>

#define _USE_MATH_DEFINES
#include <math.h>

#define MUTEX_ACCESSOR_IMPL(type, label) \
    bool Kobuki::label(type& label, bool block) \
    { \
        if (!ok()) return false;\
        const int timeout = block ? std::numeric_limits<int>::max() : 0;\
        struct pollfd fd;\
        fd.fd = m_##label.efd;\
        fd.events = POLLIN;\
        const int updated = poll(&fd, 1, timeout);\
        if (!updated) return false;\
        std::lock_guard<std::mutex> guard(m_##label.mutex);\
        label = m_##label.field;\
        return true;\
    }

#define MUTEX_ASSIGN_IMPL(type, label) \
    void Kobuki::set_##label(const type& label) \
    { \
        std::lock_guard<std::mutex> guard(m_##label.mutex);\
        m_##label.field = label;\
    }

namespace {

} // namespace

namespace kobuki {
template <size_t N>
bool create_efds(std::array<int, N> &efds)
{
    for (size_t i = 0; i < N; ++i)
    {
        const int efd = eventfd(0, EFD_NONBLOCK);
        if (efd == -1)
        {
            log_error("Could not create event file descriptor #%lu : %s", i, strerror(errno));
            return false;
        }
        efds[i] = efd;
    }

    return true;
}

Kobuki* Kobuki::create(const char* device)
{
    FILE* file = fopen(device, "rw");
    if (!file) {
        log_error("Could not open device file %s", device);
        return nullptr;
    }

    if (fcntl(fileno(file), F_SETFD, fcntl(fileno(file), F_GETFD, 0) | O_NOCTTY | O_NDELAY)) {
        log_error("Could not set flags to file descriptor");
    }

    struct termios tty;
    if (tcgetattr(fileno(file), &tty))
    {
        log_error("Could not retrive teletype attributes");
        return nullptr;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_lflag = 0;
    tty.c_oflag = 0;

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bit chars
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~PARENB; // no parity bit
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN] = sizeof(protocol::PacketHeader);
    tty.c_cc[VTIME] = 0; // Wait forever to get VMIN bytes

    if (tcsetattr(fileno(file), TCSANOW, &tty))
    {
        log_error("Could not set new teletype attributes");
        return nullptr;
    }

    std::array<int, N_EFD> efds;
    if (!create_efds(efds))
    {
        log_error("Could not create event file descriptors");
        return nullptr;
    }

    return new Kobuki(file, efds);
}

Kobuki::Kobuki(FILE* file, const std::array<int, N_EFD> &efds)
    : m_file(file), m_run(true)
{
    m_basic_data.efd = efds[0];
    m_docking_ir.efd = efds[1];
    m_inertial_data.efd = efds[2];
    m_cliff_data.efd = efds[3];
    m_current.efd = efds[4];
    m_gyro_data.efd = efds[5];
    m_gpi.efd = efds[6];
    m_pid.efd = efds[7];

    m_reading_thread = std::thread(&Kobuki::read, this);
}

Kobuki::~Kobuki()
{
    m_run = false;
    m_reading_thread.join();
    fclose(m_file);
}

static bool find_packet_header(FILE* file)
{
    constexpr uint8_t HEADER_VALS[] = { protocol::HEADER_0_VAL, protocol::HEADER_1_VAL };
    int matches = 0;
    while (matches != sizeof(HEADER_VALS))
    {
        for (size_t i = 0; i < sizeof(HEADER_VALS); ++i, ++matches)
        {
            char buffer;
            int bytes_read = fread(&buffer, 1, 1, file);
            if (bytes_read != 1) {
                log_error("Could not read packet header");
                return false;
            }

            if (buffer != HEADER_VALS[i]) {
                matches = 0;
                break;
            }
        }
    }

    return true;
}

static bool get_packet_length(FILE* file, uint8_t &length)
{
    const int bytes_read = fread(&length, 1, 1, file);
    if (bytes_read != 1) {
        log_error("Could not read packet length");
        return false;
    }

    return true;
}

bool Kobuki::process_packet(const char* buffer, uint8_t length)
{
    for (uint8_t offset = 0; offset < length;)
    {
        auto subpayload = reinterpret_cast<const protocol::FeedbackSubPayloadHeader*>(buffer + offset);
        offset += subpayload->length + sizeof(protocol::FeedbackSubPayloadHeader::type) + sizeof(protocol::FeedbackSubPayloadHeader::length);
        bool ret = true;
        switch (subpayload->type)
        {
            case protocol::Feedback::BasicSensorData:
                ret = on_msg(*reinterpret_cast<const protocol::BasicSensorData*>(subpayload));
                break;
            case protocol::Feedback::Reserved_1:
                break;
            case protocol::Feedback::DockingIR:
                ret = on_msg(*reinterpret_cast<const protocol::DockingIR*>(subpayload));
            case protocol::Feedback::InertialSensor:
                ret = on_msg(*reinterpret_cast<const protocol::InertialSensorData*>(subpayload));
            case protocol::Feedback::Cliff:
                ret = on_msg(*reinterpret_cast<const protocol::CliffSensorData*>(subpayload));
            case protocol::Feedback::Current:
                ret = on_msg(*reinterpret_cast<const protocol::Current*>(subpayload));
            case protocol::Feedback::Reserved_2:
            case protocol::Feedback::Reserved_3:
            case protocol::Feedback::Reserved_4:
                break;
            case protocol::Feedback::HardwareVersion:
                ret = on_msg(*reinterpret_cast<const protocol::HardwareVersion*>(subpayload));
            case protocol::Feedback::FirmwareVersion:
                ret = on_msg(*reinterpret_cast<const protocol::FirmwareVersion*>(subpayload));
            case protocol::Feedback::Reserved_5:
                break;
            case protocol::Feedback::RawData3AxisGyro:
                ret = on_msg(*reinterpret_cast<const protocol::RawData3AxisGyro*>(subpayload));
            case protocol::Feedback::Reserved_6:
            case protocol::Feedback::Reserved_7:
                break;
            case protocol::Feedback::GeneralPurposeInput:
                ret = on_msg(*reinterpret_cast<const protocol::GeneralPurposeInput*>(subpayload));
            case protocol::Feedback::Reserved_8:
            case protocol::Feedback::Reserved_9:
                break;
            case protocol::Feedback::UDID:
                ret = on_msg(*reinterpret_cast<const protocol::UniqueDeviceIdentifier*>(subpayload));
            case protocol::Feedback::Reserved_10:
                break;
            case protocol::Feedback::ControllerInfo:
                ret = on_msg(*reinterpret_cast<const protocol::ControllerInfo*>(subpayload));
        }

        if (!ret) {
            log_error("An error occured while processing a subpayload");
            return false;
        }
    }

    return true;
}

void Kobuki::read()
{
    while (ok())
    {
        if (!find_packet_header(m_file))
        {
            log_error("Could not find packet header");
            m_run = false;
            return;
        }

        uint8_t packet_length;
        if (!get_packet_length(m_file, packet_length))
        {
            log_error("Could not read the packet length after the in the header");
            m_run = false;
            return;
        }

        char buffer[4096];
        const int bytes_read = fread(buffer, 1, packet_length, m_file);
        if (bytes_read != packet_length) {
            log_error("Could not read packet payload. Read %ld instead of %ld", bytes_read, static_cast<int>(packet_length));
            m_run = false;
            return;
        }

        if (!process_packet(buffer, packet_length))
        {
            log_error("Could not process packet");
            m_run = false;
            return;
        }
    }
}

MUTEX_ACCESSOR_IMPL(BasicData, basic_data);
MUTEX_ACCESSOR_IMPL(DockingIR, docking_ir);
MUTEX_ACCESSOR_IMPL(InertialData, inertial_data);
MUTEX_ACCESSOR_IMPL(CliffData, cliff_data);
MUTEX_ACCESSOR_IMPL(Current, current);
MUTEX_ACCESSOR_IMPL(GyroData, gyro_data);
MUTEX_ACCESSOR_IMPL(GeneralPurposeInput, gpi);
MUTEX_ACCESSOR_IMPL(PID, pid);

MUTEX_ASSIGN_IMPL(BasicData, basic_data);
MUTEX_ASSIGN_IMPL(DockingIR, docking_ir);
MUTEX_ASSIGN_IMPL(InertialData, inertial_data);
MUTEX_ASSIGN_IMPL(CliffData, cliff_data);
MUTEX_ASSIGN_IMPL(Current, current);
MUTEX_ASSIGN_IMPL(GyroData, gyro_data);
MUTEX_ASSIGN_IMPL(GeneralPurposeInput, gpi);
MUTEX_ASSIGN_IMPL(PID, pid);

bool Kobuki::on_msg(const protocol::BasicSensorData& basic_sensor_data)
{
    BasicData basic_data;
    memset(&basic_data, sizeof(BasicData), 0);

    basic_data.timestamp_ms = basic_sensor_data.timestamp_ms;

    basic_data.left_data.bumped = basic_sensor_data.bumper & protocol::Side::Left;
    basic_data.right_data.bumped = basic_sensor_data.bumper & protocol::Side::Right;
    basic_data.center_data.bumped = basic_sensor_data.bumper & protocol::Side::Center;

    basic_data.left_data.wheel_dropped = basic_sensor_data.wheel_drop & protocol::Wheel::Left;
    basic_data.right_data.wheel_dropped = basic_sensor_data.wheel_drop & protocol::Wheel::Right;

    basic_data.left_data.cliff_sensed = basic_sensor_data.cliff & protocol::Side::Left;
    basic_data.right_data.cliff_sensed = basic_sensor_data.cliff & protocol::Side::Right;
    basic_data.center_data.cliff_sensed = basic_sensor_data.cliff & protocol::Side::Center;

    basic_data.encoder_left = basic_sensor_data.left_encoder;
    basic_data.encoder_right = basic_sensor_data.right_encoder;

    basic_data.pwm_left = basic_sensor_data.left_pwm;
    basic_data.pwm_right = basic_sensor_data.right_pwm;

    basic_data.buttons.set(0, basic_sensor_data.button & protocol::Button::Button_0);
    basic_data.buttons.set(1, basic_sensor_data.button & protocol::Button::Button_1);
    basic_data.buttons.set(2, basic_sensor_data.button & protocol::Button::Button_2);

    basic_data.is_charged = basic_sensor_data.charger & static_cast<protocol::Charger>((protocol::Charger::DockingCharged | protocol::Charger::AdapterCharged));
    basic_data.is_charging = basic_sensor_data.charger & static_cast<protocol::Charger>((protocol::Charger::DockingCharging | protocol::Charger::AdapterCharged));
    basic_data.is_using_adapter = basic_sensor_data.charger & static_cast<protocol::Charger>((protocol::Charger::AdapterCharged | protocol::Charger::AdapterCharging));
    basic_data.is_docked = basic_sensor_data.charger & static_cast<protocol::Charger>((protocol::Charger::DockingCharged | protocol::Charger::DockingCharging));

    basic_data.battery_voltage = protocol::BATTERY_VOLTAGE_RES * basic_sensor_data.battery_voltage;

    basic_data.left_data.overcurrent = basic_sensor_data.overcurrent & protocol::Wheel::Left;
    basic_data.right_data.overcurrent = basic_sensor_data.overcurrent & protocol::Wheel::Right;

    set_basic_data(basic_data);

    return true;
}

bool Kobuki::on_msg(const protocol::DockingIR& docking_ir)
{
    DockingIR ir;
    memset(&ir, sizeof(DockingIR), 0);

    ir.left.near_left = docking_ir.left & protocol::Signal::NearLeft;
    ir.left.near_right = docking_ir.left & protocol::Signal::NearRight;
    ir.left.near_center = docking_ir.left & protocol::Signal::NearCenter;

    ir.right.near_left = docking_ir.right & protocol::Signal::NearLeft;
    ir.right.near_right = docking_ir.right & protocol::Signal::NearRight;
    ir.right.near_center = docking_ir.right & protocol::Signal::NearCenter;

    ir.center.near_left = docking_ir.center & protocol::Signal::NearLeft;
    ir.center.near_right = docking_ir.center & protocol::Signal::NearRight;
    ir.center.near_center = docking_ir.center & protocol::Signal::NearCenter;

    set_docking_ir(ir);

    return true;
}

bool Kobuki::on_msg(const protocol::InertialSensorData& inertial_sensor_data)
{
    InertialData inertial_data;
    memset(&inertial_data, sizeof(InertialData), 0);

    inertial_data.angle = inertial_sensor_data.angle;
    inertial_data.angle_rate = inertial_sensor_data.angle_rate;

    set_inertial_data(inertial_data);

    return true;
}

bool Kobuki::on_msg(const protocol::CliffSensorData& cliff_sensor_data)
{
    CliffData cliff_data;
    memset(&cliff_data, sizeof(CliffData), 0);

    constexpr double DAC = 3.3 / 4096;
    cliff_data.range_left = DAC * cliff_sensor_data.left_voltage;
    cliff_data.range_right = DAC * cliff_sensor_data.right_voltage;
    cliff_data.range_center = DAC * cliff_sensor_data.center_voltage;

    set_cliff_data(cliff_data);

    return true;
}

bool Kobuki::on_msg(const protocol::Current& current)
{
    Current cur;
    memset(&cur, sizeof(Current), 0);

    constexpr double mA10 = 0.01;
    cur.current_left = mA10 * current.left_current;
    cur.current_right = mA10 * current.right_current;

    set_current(cur);

    return true;
}

bool Kobuki::on_msg(const protocol::HardwareVersion& hardware_version)
{
    char buffer[256] = {0};
    snprintf(buffer, sizeof(buffer), "%ld.%ld.%ld",
            static_cast<int>(hardware_version.major_1),
            static_cast<int>(hardware_version.minor_1),
            static_cast<int>(hardware_version.patch)
    );

    set_hardware_version(buffer);

    return true;
}

bool Kobuki::on_msg(const protocol::FirmwareVersion& firmware_version)
{
    char buffer[256] = {0};
    snprintf(buffer, sizeof(buffer), "%ld.%ld.%ld",
            static_cast<int>(firmware_version.major_1),
            static_cast<int>(firmware_version.minor_1),
            static_cast<int>(firmware_version.patch)
    );

    set_firmware_version(buffer);

    return true;
}

bool Kobuki::on_msg(const protocol::RawData3AxisGyro& raw_gyro_data)
{
    GyroData gyro_data;
    memset(&gyro_data, sizeof(GyroData), 0);

    const size_t number_of_entries = raw_gyro_data.n / 3;
    // Skipping all entries except the last one
    auto &raw_gyro_data_entry = *reinterpret_cast<const protocol::RawData3AxisGyroEntry*>(reinterpret_cast<char*>(&raw_gyro_data) + sizeof(protocol::RawData3AxisGyro) + (number_of_entries - 1) * sizeof(RawData3AxisGyroEntry));

    constexpr double DIGITS_TO_DPS = 0.00875;
    gyro_data.wx = -DIGITS_TO_DPS * raw_gyro_data_entry.y;
    gyro_data.wy = DIGITS_TO_DPS * raw_gyro_data_entry.x;
    gyro_data.wz = DIGITS_TO_DPS * raw_gyro_data_entry.z;

    set_gyro_data(gyro_data);

    return true;
}

bool Kobuki::on_msg(const protocol::GeneralPurposeInput& gpi)
{
    return true;
}

bool Kobuki::on_msg(const protocol::UniqueDeviceIdentifier& udid)
{
    return true;
}

bool Kobuki::on_msg(const protocol::ControllerInfo& controller_info)
{
    return true;
}

// TODO : Move to another file
double Kobuki::ticks_to_meters(uint16_t ticks) {
    constexpr int ticks_per_revolution = 52;
    constexpr double gear_ratio = 6545.0 / 132;
    constexpr double wheel_radius_m = 0.034;

    return ticks * wheel_radius_m * 2 * M_PI / ticks_per_revolution / gear_ratio;
}

} // namespace kobuki
