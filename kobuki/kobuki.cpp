#include "protocol.h"

#include "messages.h"

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>

#define _USE_MATH_DEFINES
#include <math.h>

namespace {
constexpr int ticks_per_revolution = 52;
constexpr double gear_ratio = 6545 / 132;
constexpr double wheel_radius_m = 0.034;

} // namespace

namespace kobuki {

Kobuki* Kobuki::create(const char* device = "/dev/kobuki")
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

    tty.c_cc[VMIN] = sizeof(PacketHeader); // Need to read 64 bytes at least before returning
    tty.c_cc[VTIME] = 0; // Wait forever to get VMIN bytes

    if (tcsetattr(fileno(file), TCSANOW, &tty))
    {
        log_error("Could not set new teletype attributes");
        return nullptr;
    }

    return new Kobuki(file);
}

Kobuki::Kobuki(FILE* file)
    : m_file(file)
{
}

Kobuki::~Kobuki()
{
    fclose(m_file);
}

double Kobuki::ticks_to_meters(uint16_t ticks) {
    return ticks * wheel_radius_m * 2 * M_PI / ticks_per_revolution / gear_ratio;
}

} // namespace kobuki
