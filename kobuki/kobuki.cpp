#include "kobuki.h"

#define _USE_MATH_DEFINES
#include <math.h>

namespace {
constexpr int ticks_per_revolution = 52;
constexpr double gear_ratio = 6545 / 132;
constexpr double wheel_radius_m = 0.034;

} // namespace

namespace kobuki {
double Kobuki::ticks_to_meters(uint16_t ticks) {
    return ticks * wheel_radius_m * 2 * M_PI / ticks_per_revolution / gear_ratio;
}

} // namespace kobuki
