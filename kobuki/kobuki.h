#pragma once

#include "messages.h"

#include <array>
#include <string>
#include <thread>
#include <mutex>

#define MUTEX_ACCESSOR(type, label) bool label(type& label, bool block)

namespace kobuki {

template <typename T>
struct EventField {
    int efd;
    std::mutex mutex;
    T field;
};

class Kobuki {
public:
    static Kobuki* create(const char* device = "/dev/kobuki");
    ~Kobuki();

    bool ok() const { return m_run; }

    MUTEX_ACCESSOR(BasicData, basic_data);
    MUTEX_ACCESSOR(DockingIR, docking_ir);
    MUTEX_ACCESSOR(InertialData, inertial_data);
    MUTEX_ACCESSOR(CliffData, cliff_data);
    MUTEX_ACCESSOR(Current, current);
    MUTEX_ACCESSOR(GyroData, gyro_data);
    MUTEX_ACCESSOR(GeneralPurposeInput, gpi);
    MUTEX_ACCESSOR(PID, pid);

    // TODO : Move to another file
    static double ticks_to_meters(uint16_t ticks);
private:
    static constexpr size_t N_EFD = 8;

    Kobuki(FILE* file, const std::array<int, N_EFD> &efds);

    void read();

private:
    FILE* m_file;
    std::thread m_reading_thread;
    bool m_run;

    EventField<BasicData> m_basic_data;
    EventField<DockingIR> m_docking_ir;
    EventField<InertialData> m_inertial_data;
    EventField<CliffData> m_cliff_data;
    EventField<Current> m_current;
    EventField<GyroData> m_gyro_data;
    EventField<GeneralPurposeInput> m_gpi;
    EventField<PID> m_pid;
};

} // namespace kobuki
