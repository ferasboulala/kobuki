#include "kobuki.h"

#include "log.h"

#include <string>
#include <thread>
#include <chrono>

const char* strbutton(bool pressed)
{
    return pressed ? "x" : " ";
}

int main() {
    using namespace kobuki;
    Kobuki* robot = Kobuki::create();
    if (!robot) {
        log_fatal("Could not create kobuki instance");
    }

    const std::string firmware_version = robot->get_firmware_version();
    const std::string hardware_version = robot->get_hardware_version();

    printf("firmware : %s\n", firmware_version.c_str());
    printf("hardware : %s\n", hardware_version.c_str());

    while (true)
    {
        BasicData basic_data;
        if (!robot->get_basic_data(basic_data, true))
        {
            log_error("Could not retrieve data");
            break;
        }

        const std::string status = 
            basic_data.is_charging ? "Charging" : "Discharging";

        printf("[%s] - B0[%s] B1[%s] B2[%s]\n",
                status.c_str(),
                strbutton(basic_data.buttons[0]),
                strbutton(basic_data.buttons[1]),
                strbutton(basic_data.buttons[2])
        );

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    delete robot;
    return 0;
}
