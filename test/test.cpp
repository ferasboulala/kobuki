#include "kobuki.h"

#include "log.h"

#include <thread>
#include <chrono>

int main() {
    using namespace kobuki;
    Kobuki* robot = Kobuki::create();
    if (!robot) {
        log_fatal("Could not create kobuki instance");
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    delete robot;
    return 0;
}
