#include "kobuki.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/eventfd.h>
#include <sys/poll.h>
#include <termios.h>
#include <unistd.h>

#include "log.h"
#include "messages.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define MUTEX_ACCESSOR_IMPL(type, label)                                 \
    bool Kobuki::get_##label(type& label, bool block)                    \
    {                                                                    \
        if (!ok()) return false;                                         \
        const int timeout = block ? std::numeric_limits<int>::max() : 0; \
        struct pollfd fd;                                                \
        fd.fd = m_##label.efd;                                           \
        fd.events = POLLIN;                                              \
        const int updated = poll(&fd, 1, timeout);                       \
        if (updated == -1)                                               \
        {                                                                \
            log_error("Could not read field " #type);                    \
            m_run = false;                                               \
            return false;                                                \
        }                                                                \
        if (!updated) return false;                                      \
        std::lock_guard<std::mutex> guard(m_##label.mutex);              \
        label = m_##label.field;                                         \
        return true;                                                     \
    }

#define MUTEX_ASSIGN_IMPL(type, label)                                                     \
    void Kobuki::set_##label(const type& label)                                            \
    {                                                                                      \
        std::lock_guard<std::mutex> guard(m_##label.mutex);                                \
        m_##label.field = label;                                                           \
        const int64_t buffer = 1;                                                          \
        const int bytes_written = write(m_##label.efd, &buffer, sizeof(int64_t));          \
        if (bytes_written != sizeof(buffer))                                               \
        {                                                                                  \
            log_error("Could not write to " #type " label. Wrote %ld instead of %lu : %s", \
                      bytes_written,                                                       \
                      sizeof(buffer),                                                      \
                      strerror(errno));                                                    \
            m_run = false;                                                                 \
        }                                                                                  \
    }

namespace
{
}  // namespace

namespace kobuki
{
template <size_t N>
bool create_efds(std::array<int, N>& efds)
{
    for (size_t i = 0; i < N; ++i)
    {
        const int efd = eventfd(0, 0);
        if (efd == -1)
        {
            log_error("Could not create event file descriptor #%lu : %s", i, strerror(errno));
            return false;
        }
        efds[i] = efd;
    }

    return true;
}

template <typename T>
T prepare_message(protocol::Command type)
{
    T msg;
    memset(&msg, 0, sizeof(T));
    msg.type = type;
    msg.length = sizeof(T) - sizeof(protocol::CommandSubPayloadHeader);

    return msg;
}

Kobuki* Kobuki::create(const char* device)
{
    FILE* file = fopen(device, "rb+");
    if (!file)
    {
        log_error("Could not open device file %s", device);
        return nullptr;
    }

    if (fcntl(fileno(file), F_SETFD, fcntl(fileno(file), F_GETFD, 0) | O_NOCTTY | O_NDELAY))
    {
        log_error("Could not set flags to file descriptor");
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    // if (tcgetattr(fileno(file), &tty))
    //{
    //    log_error("Could not retrive teletype attributes");
    //    return nullptr;
    //}

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // http://docs.ros.org/en/noetic/api/ecl_devices/html/serial__pos_8cpp_source.html
    // TODO : Add a lock by following the same steps as above (the link)
    // tty.c_lflag = 0;
    // tty.c_oflag = 0;
    // tty.c_cflag = 0;
    // tty.c_iflag = 0;

    // tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8 bit chars
    tty.c_cflag &= ~CSTOPB;                      // 1 stop bit
    tty.c_cflag &= ~PARENB;                      // no parity bit
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN] = sizeof(protocol::PacketHeader);
    tty.c_cc[VTIME] = 0;  // Wait forever to get VMIN bytes

    if (tcsetattr(fileno(file), TCSAFLUSH, &tty))
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

Kobuki::Kobuki(FILE* file, const std::array<int, N_EFD>& efds)
    : m_file(file), m_run(true), m_cached_output(static_cast<protocol::DigitalOutput>(0))
{
    m_basic_data.efd = efds[0];
    m_docking_ir.efd = efds[1];
    m_inertial_data.efd = efds[2];
    m_cliff_data.efd = efds[3];
    m_current.efd = efds[4];
    m_gyro_data.efd = efds[5];
    m_gpi.efd = efds[6];
    m_pid.efd = efds[7];

    m_reading_thread = std::thread(&Kobuki::spin, this);
    request_identifiers();
    set_leds(false, false, false,
             false);  // Will turn everything down (power and digital output too)
}

Kobuki::~Kobuki()
{
    m_run = false;
    m_reading_thread.join();
    fclose(m_file);
}

bool Kobuki::find_packet_header()
{
    constexpr uint8_t HEADER_VALS[] = {protocol::HEADER_0_VAL, protocol::HEADER_1_VAL};
    int matches = 0;
    while (matches != sizeof(HEADER_VALS))
    {
        for (size_t i = 0; i < sizeof(HEADER_VALS); ++i, ++matches)
        {
            unsigned char buffer;
            int bytes_read = fread(&buffer, 1, 1, m_file);
            if (bytes_read != 1)
            {
                log_error("Could not read packet header");
                return false;
            }

            if (buffer != HEADER_VALS[i])
            {
                matches = 0;
                break;
            }
        }
    }

    return true;
}

bool Kobuki::process_packet(const char* buffer, uint8_t length)
{
    uint8_t offset;
    for (offset = 0; offset < length;)
    {
        if (sizeof(protocol::FeedbackSubPayloadHeader) + offset >= length)
        {
            log_error("Not enough remaining bytes for header : %lu vs %d",
                      sizeof(protocol::FeedbackSubPayloadHeader),
                      static_cast<int>(length - offset));
            return false;
        }

        auto subpayload = reinterpret_cast<const protocol::FeedbackSubPayloadHeader*>(buffer + offset);
        offset += sizeof(protocol::FeedbackSubPayloadHeader);
        if (offset >= length)
        {
            log_error("Not enough remaining bytes for the subpayload : %d vs %d",
                      static_cast<int>(subpayload->length),
                      static_cast<int>(offset - length));
            return false;
        }

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
                break;
            case protocol::Feedback::InertialSensor:
                ret = on_msg(*reinterpret_cast<const protocol::InertialSensorData*>(subpayload));
                break;
            case protocol::Feedback::Cliff:
                ret = on_msg(*reinterpret_cast<const protocol::CliffSensorData*>(subpayload));
                break;
            case protocol::Feedback::Current:
                ret = on_msg(*reinterpret_cast<const protocol::Current*>(subpayload));
                break;
            case protocol::Feedback::Reserved_2:
            case protocol::Feedback::Reserved_3:
            case protocol::Feedback::Reserved_4:
                break;
            case protocol::Feedback::HardwareVersion:
                ret = on_msg(*reinterpret_cast<const protocol::HardwareVersion*>(subpayload));
                break;
            case protocol::Feedback::FirmwareVersion:
                ret = on_msg(*reinterpret_cast<const protocol::FirmwareVersion*>(subpayload));
                break;
            case protocol::Feedback::Reserved_5:
                break;
            case protocol::Feedback::RawData3AxisGyro:
                ret = on_msg(*reinterpret_cast<const protocol::RawData3AxisGyro*>(subpayload));
                break;
            case protocol::Feedback::Reserved_6:
            case protocol::Feedback::Reserved_7:
                break;
            case protocol::Feedback::GeneralPurposeInput:
                ret = on_msg(*reinterpret_cast<const protocol::GeneralPurposeInput*>(subpayload));
                break;
            case protocol::Feedback::Reserved_8:
            case protocol::Feedback::Reserved_9:
                break;
            case protocol::Feedback::UDID:
                ret = on_msg(*reinterpret_cast<const protocol::UniqueDeviceIdentifier*>(subpayload));
                break;
            case protocol::Feedback::Reserved_10:
                break;
            case protocol::Feedback::ControllerInfo:
                ret = on_msg(*reinterpret_cast<const protocol::ControllerInfo*>(subpayload));
                break;
            default:
                log_error("Unrecognized subpaylaod type %d", static_cast<int>(subpayload->type));
                return false;
        }

        if (!ret)
        {
            log_error("An error occured while processing a subpayload");
            return false;
        }

        offset += subpayload->length;
    }

    return offset == length;
}

uint8_t Kobuki::checksum(uint8_t packet_length, const char* buffer)
{
    uint8_t cs = packet_length;
    for (int i = 0; i < packet_length; ++i)
    {
        cs ^= buffer[i];
    }

    return cs;
}

bool Kobuki::validate_checksum(uint8_t packet_length, const char* buffer)
{
    uint8_t cs;
    if (fread(&cs, 1, 1, m_file) != sizeof(cs))
    {
        log_error("Could not read checksum value");
        return false;
    }

    return checksum(packet_length, buffer) ^ cs ? false : true;
}

void Kobuki::spin()
{
    while (ok())
    {
        if (!find_packet_header())
        {
            log_error("Could not find packet header");
            m_run = false;
            return;
        }

        uint8_t packet_length;
        int bytes_read = fread(&packet_length, 1, sizeof(packet_length), m_file);
        if (bytes_read != sizeof(packet_length))
        {
            log_error("Could not read the packet length");
            m_run = false;
            return;
        }

        char buffer[4096];
        bytes_read = fread(buffer, 1, packet_length, m_file);
        if (bytes_read != packet_length)
        {
            log_error(
                "Could not read packet payload. Read %ld instead of %ld", bytes_read, static_cast<int>(packet_length));
            m_run = false;
            return;
        }

        if (!validate_checksum(packet_length, buffer))
        {
            log_error("Failed checksum. Skipping packet");
            continue;
        }

        if (!process_packet(buffer, packet_length))
        {
            log_error("Could not process packet");
            m_run = false;
            return;
        }
    }
}

MUTEX_ACCESSOR_IMPL(BasicData, basic_data)
MUTEX_ACCESSOR_IMPL(DockingIR, docking_ir)
MUTEX_ACCESSOR_IMPL(InertialData, inertial_data)
MUTEX_ACCESSOR_IMPL(CliffData, cliff_data)
MUTEX_ACCESSOR_IMPL(Current, current)
MUTEX_ACCESSOR_IMPL(GyroData, gyro_data)
MUTEX_ACCESSOR_IMPL(GeneralPurposeInput, gpi)
MUTEX_ACCESSOR_IMPL(PID, pid)

std::string Kobuki::get_hardware_version() const { return m_hardware_version; }

std::string Kobuki::get_firmware_version() const { return m_firmware_version; }

UDID Kobuki::get_udid() const { return m_udid; }

template <typename T>
void Kobuki::send_msg(const T& msg)
{
    protocol::PacketHeader header;
    header.length = sizeof(T);
    if (write(fileno(m_file), &header, sizeof(header)) != sizeof(header))
    {
        log_error("Could not send packet header : %s", strerror(errno));
        m_run = false;
        return;
    }

    if (write(fileno(m_file), &msg, sizeof(T)) != sizeof(T))
    {
        log_error("Could not send message : %s", strerror(errno));
        m_run = false;
        return;
    }

    const uint8_t cs = checksum(header.length, reinterpret_cast<const char*>(&msg));
    if (write(fileno(m_file), &cs, 1) != sizeof(cs))
    {
        log_error("Could not send checksum : %s", strerror(errno));
        m_run = false;
    }
}

void Kobuki::set_motion(double velocity, double radius)
{
    auto msg = prepare_message<protocol::MotionMessage>(protocol::Command::Motion);
    msg.velocity = velocity * 1000;  // m/s to mm/s
    msg.radius = radius * 1000;      // m to mm

    send_msg(msg);
}

void Kobuki::set_sound(double frequency, double duration)
{
    auto msg = prepare_message<protocol::SoundMessage>(protocol::Command::Sound);
    constexpr double A = 0.00000275;
    msg.period = 1.0 / (frequency * A);
    msg.duration = duration * 1000;  // s to ms

    send_msg(msg);
}

void Kobuki::set_sound_sequence(SoundSequence sequence)
{
    auto msg = prepare_message<protocol::SoundSequence>(protocol::Command::SoundSequence);
    msg.sequence_number = static_cast<protocol::SoundSequenceNumber>(sequence);

    send_msg(msg);
}

void Kobuki::set_leds(bool led_1_green, bool led_1_red, bool led_2_green, bool led_2_red)
{
    auto msg = prepare_message<protocol::GeneralPurposeOutput>(protocol::Command::GeneralPurposeOutput);
    msg.digital_output = m_cached_output;
    m_cached_output &= ~(protocol::DigitalOutput::LED_1_Red | protocol::DigitalOutput::LED_1_Green |
                         protocol::DigitalOutput::LED_2_Red | protocol::DigitalOutput::LED_2_Green);

    if (led_1_green) m_cached_output |= protocol::DigitalOutput::LED_1_Green;
    if (led_1_red) m_cached_output |= protocol::DigitalOutput::LED_1_Red;
    if (led_2_green) m_cached_output |= protocol::DigitalOutput::LED_2_Green;
    if (led_2_red) m_cached_output |= protocol::DigitalOutput::LED_2_Red;

    msg.digital_output = m_cached_output;

    send_msg(msg);
}

void Kobuki::set_power_output(bool power_3_3, bool power_5, bool power_12_5, bool power_12_1_5)
{
    auto msg = prepare_message<protocol::GeneralPurposeOutput>(protocol::Command::GeneralPurposeOutput);
    msg.digital_output = m_cached_output;
    m_cached_output &= ~(protocol::DigitalOutput::Power_3_3 | protocol::DigitalOutput::Power_5 |
                         protocol::DigitalOutput::Power_12_5 | protocol::DigitalOutput::Power_12_1_5);
    if (power_3_3) m_cached_output |= protocol::DigitalOutput::Power_3_3;
    if (power_5) m_cached_output |= protocol::DigitalOutput::Power_5;
    if (power_12_5) m_cached_output |= protocol::DigitalOutput::Power_12_5;
    if (power_12_1_5) m_cached_output |= protocol::DigitalOutput::Power_12_1_5;

    msg.digital_output = m_cached_output;

    send_msg(msg);
}

MUTEX_ASSIGN_IMPL(BasicData, basic_data)
MUTEX_ASSIGN_IMPL(DockingIR, docking_ir)
MUTEX_ASSIGN_IMPL(InertialData, inertial_data)
MUTEX_ASSIGN_IMPL(CliffData, cliff_data)
MUTEX_ASSIGN_IMPL(Current, current)
MUTEX_ASSIGN_IMPL(GyroData, gyro_data)
MUTEX_ASSIGN_IMPL(GeneralPurposeInput, gpi)
MUTEX_ASSIGN_IMPL(PID, pid)

bool Kobuki::on_msg(const protocol::BasicSensorData& basic_sensor_data)
{
    BasicData basic_data;

    basic_data.timestamp_ms = basic_sensor_data.timestamp_ms;

    basic_data.left_data.bumped = basic_sensor_data.bumper & protocol::Side::Left;
    basic_data.right_data.bumped = basic_sensor_data.bumper & protocol::Side::Right;
    basic_data.center_data.bumped = basic_sensor_data.bumper & protocol::Side::Center;

    basic_data.left_data.wheel_dropped = basic_sensor_data.wheel_drop & protocol::Wheel::Left;
    basic_data.right_data.wheel_dropped = basic_sensor_data.wheel_drop & protocol::Wheel::Right;

    basic_data.left_data.cliff_sensed = basic_sensor_data.cliff & protocol::Side::Left;
    basic_data.right_data.cliff_sensed = basic_sensor_data.cliff & protocol::Side::Right;
    basic_data.center_data.cliff_sensed = basic_sensor_data.cliff & protocol::Side::Center;

    basic_data.left_data.encoder = basic_sensor_data.left_encoder;
    basic_data.right_data.encoder = basic_sensor_data.right_encoder;

    basic_data.left_data.pwm = basic_sensor_data.left_pwm;
    basic_data.right_data.pwm = basic_sensor_data.right_pwm;

    basic_data.buttons.set(0, basic_sensor_data.button & protocol::Button::Button_0);
    basic_data.buttons.set(1, basic_sensor_data.button & protocol::Button::Button_1);
    basic_data.buttons.set(2, basic_sensor_data.button & protocol::Button::Button_2);

    basic_data.is_charged =
        basic_sensor_data.charger &
        static_cast<protocol::Charger>((protocol::Charger::DockingCharged | protocol::Charger::AdapterCharged));
    basic_data.is_charging =
        basic_sensor_data.charger &
        static_cast<protocol::Charger>((protocol::Charger::DockingCharging | protocol::Charger::AdapterCharged));
    basic_data.is_using_adapter =
        basic_sensor_data.charger &
        static_cast<protocol::Charger>((protocol::Charger::AdapterCharged | protocol::Charger::AdapterCharging));
    basic_data.is_docked =
        basic_sensor_data.charger &
        static_cast<protocol::Charger>((protocol::Charger::DockingCharged | protocol::Charger::DockingCharging));

    basic_data.battery_voltage = protocol::BATTERY_VOLTAGE_RES * basic_sensor_data.battery_voltage;

    basic_data.left_data.overcurrent = basic_sensor_data.overcurrent & protocol::Wheel::Left;
    basic_data.right_data.overcurrent = basic_sensor_data.overcurrent & protocol::Wheel::Right;

    set_basic_data(basic_data);

    return true;
}

bool Kobuki::on_msg(const protocol::DockingIR& docking_ir)
{
    DockingIR ir;

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

    inertial_data.angle = inertial_sensor_data.angle;
    inertial_data.angle_rate = inertial_sensor_data.angle_rate;

    set_inertial_data(inertial_data);

    return true;
}

bool Kobuki::on_msg(const protocol::CliffSensorData& cliff_sensor_data)
{
    CliffData cliff_data;

    constexpr double DAC = 3.3 / 4096;
    cliff_data.voltage_left = DAC * cliff_sensor_data.left_voltage;
    cliff_data.voltage_right = DAC * cliff_sensor_data.right_voltage;
    cliff_data.voltage_center = DAC * cliff_sensor_data.center_voltage;

    set_cliff_data(cliff_data);

    return true;
}

bool Kobuki::on_msg(const protocol::Current& current)
{
    Current cur;

    constexpr double mA10 = 0.01;
    cur.current_left = mA10 * current.left_current;
    cur.current_right = mA10 * current.right_current;

    set_current(cur);

    return true;
}

bool Kobuki::on_msg(const protocol::HardwareVersion& hardware_version)
{
    char buffer[256] = {0};
    snprintf(buffer,
             sizeof(buffer),
             "%d.%d.%d",
             static_cast<int>(hardware_version.major_1),
             static_cast<int>(hardware_version.minor_1),
             static_cast<int>(hardware_version.patch));

    m_hardware_version = buffer;

    return true;
}

bool Kobuki::on_msg(const protocol::FirmwareVersion& firmware_version)
{
    char buffer[256] = {0};
    snprintf(buffer,
             sizeof(buffer),
             "%d.%d.%d",
             static_cast<int>(firmware_version.major_1),
             static_cast<int>(firmware_version.minor_1),
             static_cast<int>(firmware_version.patch));

    m_firmware_version = buffer;

    return true;
}

bool Kobuki::on_msg(const protocol::RawData3AxisGyro& raw_gyro_data)
{
    GyroData gyro_data;

    const size_t number_of_entries = raw_gyro_data.n / 3;
    // Skipping all entries except the last one
    auto& raw_gyro_data_entry = *reinterpret_cast<const protocol::RawData3AxisGyroEntry*>(
        reinterpret_cast<const char*>(&raw_gyro_data) + sizeof(protocol::RawData3AxisGyro) +
        (number_of_entries - 1) * sizeof(protocol::RawData3AxisGyroEntry));

    constexpr double DIGITS_TO_DPS = 0.00875;
    gyro_data.wx = -DIGITS_TO_DPS * raw_gyro_data_entry.y;
    gyro_data.wy = DIGITS_TO_DPS * raw_gyro_data_entry.x;
    gyro_data.wz = DIGITS_TO_DPS * raw_gyro_data_entry.z;

    set_gyro_data(gyro_data);

    return true;
}

bool Kobuki::on_msg(const protocol::GeneralPurposeInput& gpi)
{
    GeneralPurposeInput general_purpose_input;

    general_purpose_input.digital_inputs.set(0, gpi.digital_input & protocol::DigitalInput::Channel_0);
    general_purpose_input.digital_inputs.set(1, gpi.digital_input & protocol::DigitalInput::Channel_1);
    general_purpose_input.digital_inputs.set(2, gpi.digital_input & protocol::DigitalInput::Channel_2);
    general_purpose_input.digital_inputs.set(3, gpi.digital_input & protocol::DigitalInput::Channel_3);

    constexpr double DAC = 3.3 / 4096;
    general_purpose_input.voltage_0 = DAC * gpi.analog_input_0;
    general_purpose_input.voltage_1 = DAC * gpi.analog_input_1;
    general_purpose_input.voltage_2 = DAC * gpi.analog_input_2;
    general_purpose_input.voltage_3 = DAC * gpi.analog_input_3;

    set_gpi(general_purpose_input);

    return true;
}

bool Kobuki::on_msg(const protocol::UniqueDeviceIdentifier& udid)
{
    m_udid.id_0 = udid.id_0;
    m_udid.id_1 = udid.id_1;
    m_udid.id_2 = udid.id_2;

    return true;
}

bool Kobuki::on_msg(const protocol::ControllerInfo& controller_info)
{
    PID pid;

    pid.P = controller_info.proportional / 1000;
    pid.I = controller_info.integral / 1000;
    pid.D = controller_info.derivate / 1000;

    set_pid(pid);

    return true;
}

void Kobuki::request_identifiers()
{
    auto request_extra = prepare_message<protocol::RequestExtra>(protocol::Command::RequestExtra);
    request_extra.flags = protocol::RequestExtraFlag::HardwareVersion | protocol::RequestExtraFlag::FirmwareVersion |
                          protocol::RequestExtraFlag::UDID;

    send_msg(request_extra);
}

// TODO : Move to another file
double Kobuki::ticks_to_meters(uint16_t ticks)
{
    constexpr int ticks_per_revolution = 52;
    constexpr double gear_ratio = 6545.0 / 132;
    constexpr double wheel_radius_m = 0.034;

    return ticks * wheel_radius_m * 2 * M_PI / ticks_per_revolution / gear_ratio;
}

}  // namespace kobuki
