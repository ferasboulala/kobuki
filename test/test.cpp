#include "kobuki.h"

#include "log.h"

#include <chrono>
#include <string>
#include <thread>

const char *strbool(bool val) { return val ? "x" : " "; }

int main() {
  using namespace kobuki;
  Kobuki *robot = Kobuki::create();
  if (!robot) {
    log_fatal("Could not create kobuki instance");
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const std::string firmware_version = robot->get_firmware_version();
  const std::string hardware_version = robot->get_hardware_version();

  printf("firmware : %s\n", firmware_version.c_str());
  printf("hardware : %s\n", hardware_version.c_str());

  robot->set_leds(false, false, false, false);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  // while (true)
  //{
  //    BasicData basic_data;
  //    if (!robot->get_basic_data(basic_data, true))
  //    {
  //        log_error("Could not retrieve data");
  //        break;
  //    }

  //    printf(
  //            "time: %d\n"
  //            "\t\tLEFT\t\tCENTER\t\tRIGHT\n"
  //            "bumper\t\t%s\t\t%s\t\t%s\n"
  //            "cliff\t\t""%s\t\t%s\t\t%s\n"
  //            "wheeldrop\t%s\t\t - \t\t%s\n"
  //            "overcurrent\t%s\t\t - \t\t%s\n"
  //            "encoder\t\t%d\t\t - \t\t%d\n"
  //            "pwm\t\t%d\t\t - \t\t%d\n",
  //            static_cast<int>(basic_data.timestamp_ms),
  //            strbool(basic_data.left_data.bumped),
  //            strbool(basic_data.right_data.bumped),
  //            strbool(basic_data.center_data.bumped),
  //            strbool(basic_data.left_data.cliff_sensed),
  //            strbool(basic_data.right_data.cliff_sensed),
  //            strbool(basic_data.center_data.cliff_sensed),
  //            strbool(basic_data.left_data.wheel_dropped),
  //            strbool(basic_data.right_data.wheel_dropped),
  //            strbool(basic_data.left_data.overcurrent),
  //            strbool(basic_data.right_data.overcurrent),
  //            static_cast<int>(basic_data.left_data.encoder),
  //            static_cast<int>(basic_data.right_data.encoder),
  //            static_cast<int>(basic_data.left_data.pwm),
  //            static_cast<int>(basic_data.right_data.pwm)
  //    );

  //    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  //}

  delete robot;
  return 0;
}
