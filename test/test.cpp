#include "kobuki.h"

#include "log.h"

int main() {
    using namespace kobuki;
    Kobuki* robot = Kobuki::create();
    if (!robot) {
        log_fatal("Could not create kobuki instance");
    }

    delete robot;
    return 0;
}
