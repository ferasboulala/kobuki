#include "kobuki.h"

#include "protocol.h"
#include "messages.h"

#include "log.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
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

namespace {
constexpr int ticks_per_revolution = 52;
constexpr double gear_ratio = 6545.0 / 132;
constexpr double wheel_radius_m = 0.034;

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

    struct termios tty;
    if (tcgetattr(fileno(file), &tty))
    {
        log_error("Could not retrive teletype attributes");
        return nullptr;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bit chars
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~PARENB; // no parity bit

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

bool find_packet_header(FILE* file)
{
    constexpr uint8_t HEADER[] = { protocol::HEADER_0_VAL, protocol::HEADER_1_VAL };
    int matches = 0;
    while (matches != sizeof(HEADER))
    {
        for (size_t i = 0; i < sizeof(HEADER); ++i, ++matches)
        {
            char buffer;
            int bytes_read = fread(&buffer, 1, 1, file);
            if (bytes_read != 1) {
                log_error("Could not read packet header");
                return false;
            }

            if (buffer != HEADER[0]) {
                matches = 0;
                break;
            }
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

// TODO : Move to another file
double Kobuki::ticks_to_meters(uint16_t ticks) {
    return ticks * wheel_radius_m * 2 * M_PI / ticks_per_revolution / gear_ratio;
}

} // namespace kobuki
