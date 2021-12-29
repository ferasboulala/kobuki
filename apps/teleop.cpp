#include <ncurses.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "kobuki.h"
#include "log.h"

double accelerate(double vel, double acc, double target)
{
    if (acc > 0)
    {
        return std::min(vel + acc, target);
    }
    return std::max(vel + acc, target);
}

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

    std::string firmware_version;
    std::string hardware_version;
    while (firmware_version.empty() || hardware_version.empty())
    {
        firmware_version = robot->get_firmware_version();
        hardware_version = robot->get_hardware_version();
    }
    printf("firmware : %s\n", firmware_version.c_str());
    printf("hardware : %s\n", hardware_version.c_str());

    double vel = 0;
    constexpr double dt = 0.05;
    constexpr double MAX_VEL = MAX_TRANS_VELOCITY / 2;
    constexpr double ACC = MAX_VEL * dt;

    initscr();
    timeout(0);
    bool quit = false;
    robot->set_power_output(false, false, true, true);
    while (!quit)
    {
        int key = getch();
        switch (key)
        {
            case 119:  // W
                vel = accelerate(vel, ACC, MAX_VEL);
                robot->pure_translation(vel);
                robot->set_leds(true, false, false, false);
                break;
            case 115:  // S
                vel = accelerate(vel, -ACC, -MAX_VEL);
                robot->pure_translation(vel);
                robot->set_leds(false, true, false, false);
                break;
            case 97:  // A
                if (vel)
                {
                    vel = accelerate(vel, vel > 0 ? -ACC : ACC, 0);
                    robot->pure_translation(vel);
                }
                else
                    robot->pure_rotation(MAX_ROT_VELOCITY_SMOOTH / 2);
                robot->set_leds(false, true, true, false);
                break;
            case 100:  // D
                if (vel)
                {
                    vel = accelerate(vel, vel > 0 ? -ACC : ACC, 0);
                    robot->pure_translation(vel);
                }
                else
                    robot->pure_rotation(-MAX_ROT_VELOCITY_SMOOTH / 2);
                robot->set_leds(true, false, false, true);
                break;
            case 113:  // Q
                robot->set_motion(0, 0);
                quit = true;
                robot->set_leds(false, false, false, false);
                break;
            default:
                vel = accelerate(vel, vel > 0 ? -ACC : ACC, 0);
                robot->pure_translation(vel);
                robot->set_leds(false, true, false, true);
                break;
        }
        flushinp();
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }
    endwin();

    return 0;
}
