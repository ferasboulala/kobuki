#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <ncurses.h>

#include "kobuki.h"
#include "log.h"

const char* strbool(bool val) { return val ? "x" : " "; }

void print_basic_data(const kobuki::BasicData& basic_data)
{
    printf(
        "time: %d\n"
        "\t\tLEFT\t\tCENTER\t\tRIGHT\n"
        "bumper\t\t%s\t\t%s\t\t%s\n"
        "cliff\t\t"
        "%s\t\t%s\t\t%s\n"
        "wheeldrop\t%s\t\t - \t\t%s\n"
        "overcurrent\t%s\t\t - \t\t%s\n"
        "encoder\t\t%d\t\t - \t\t%d\n"
        "pwm\t\t%d\t\t - \t\t%d\n",
        static_cast<int>(basic_data.timestamp_ms),
        strbool(basic_data.left_data.bumped),
        strbool(basic_data.right_data.bumped),
        strbool(basic_data.center_data.bumped),
        strbool(basic_data.left_data.cliff_sensed),
        strbool(basic_data.right_data.cliff_sensed),
        strbool(basic_data.center_data.cliff_sensed),
        strbool(basic_data.left_data.wheel_dropped),
        strbool(basic_data.right_data.wheel_dropped),
        strbool(basic_data.left_data.overcurrent),
        strbool(basic_data.right_data.overcurrent),
        static_cast<int>(basic_data.left_data.encoder),
        static_cast<int>(basic_data.right_data.encoder),
        static_cast<int>(basic_data.left_data.pwm),
        static_cast<int>(basic_data.right_data.pwm));
}

int main()
{
    using namespace kobuki;
    auto robot = std::unique_ptr<Kobuki>(Kobuki::create());
    if (!robot)
    {
        log_fatal("Could not create kobuki instance");
    }

    std::string firmware_version = robot->get_firmware_version();
    std::string hardware_version = robot->get_hardware_version();
    while (firmware_version.empty() || hardware_version.empty())
    {
        firmware_version = robot->get_firmware_version();
        hardware_version = robot->get_hardware_version();
    }
    printf("firmware : %s\n", firmware_version.c_str());
    printf("hardware : %s\n", hardware_version.c_str());

    // std::this_thread::sleep_for(std::chrono::seconds(1));
    initscr();
    timeout(-1);
    bool quit = false;
    while (!quit)
    {
        refresh();
        BasicData basic_data;
        if (robot->get_basic_data(basic_data, false))
        {
            //print_basic_data(basic_data);
        }
        const int key = getch();
        log_info("%d", key);
        switch (key)
        {
            case 65: // up
                robot->pure_translation(MAX_TRANS_VELOCITY / 3);
                break;
            case 68: // left
                robot->pure_rotation(MAX_ROT_VELOCITY_SMOOTH);
                break;
            case 67: // right:
                robot->pure_rotation(-MAX_ROT_VELOCITY_SMOOTH);
                break;
            case 113: // q
                quit = true;
                break;
            default:
                log_info("shhh");
                robot->set_motion(0, 0);
                break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    endwin();

    return 0;
}
