#pragma once

#include "messages.h"

#include <string>
#include <optional>

namespace kobuki {

template <typename T>
struct EventField {
    int efd;
    std::optional<T> field;
};

class Kobuki {
public:
    static Kobuki* create(const char* device = "/dev/kobuki");
    ~Kobuki();

    static double ticks_to_meters(uint16_t ticks);
private:
    Kobuki(FILE* file);

private:
    FILE* m_file;

    EventField<BasicData> m_basic_data;
    EventField<DockingIR> m_docking_signals;
    EventField<InertialData> m_intertial_data;
    EventField<CliffData> m_cliff_data;
    EventField<Current> m_current;
    EventField<GyroData> m_gyro_data;
    EventField<GeneralPurposeInput> m_gpi;
    EventField<PID> m_pid;
};

} // namespace kobuki
