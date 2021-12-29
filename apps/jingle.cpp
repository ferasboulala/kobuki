#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "kobuki.h"
#include "log.h"

const std::vector<std::tuple<double, double>> happy_birthday = {
    {392, 0.25}, {392, 0.25}, {440, 0.25}, {392, 0.25}, {600, 0.25}, {494, 0.25}};

int main()
{
    using namespace kobuki;
    auto robot = std::unique_ptr<Kobuki>(Kobuki::create());
    if (!robot)
    {
        log_fatal("Could not create kobuki instance");
    }

    std::string firmware_version;
    std::string hardware_version;
    while (firmware_version.empty() || hardware_version.empty())
    {
        firmware_version = robot->get_firmware_version();
        hardware_version = robot->get_hardware_version();
    }
    printf("firmware : %s\n", firmware_version.c_str());
    printf("hardware : %s\n", hardware_version.c_str());

    for (const auto &[f, d] : happy_birthday)
    {
        robot->set_sound(f, d);
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(d * 1000)));
    }

    return 0;
}
