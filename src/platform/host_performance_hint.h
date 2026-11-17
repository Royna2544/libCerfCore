#pragma once

#include <cstdint>

namespace cerfcore {

namespace host_platform {

class HostPerformanceHint {
public:
    HostPerformanceHint() = default;
    ~HostPerformanceHint();
    HostPerformanceHint(const HostPerformanceHint&)            = delete;
    HostPerformanceHint& operator=(const HostPerformanceHint&) = delete;

    bool OpenForCurrentThread(int64_t target_ns);
    void ReportWork(int64_t actual_ns);
    void Close();
    bool Open() const { return session_ != nullptr; }

private:
    void* session_ = nullptr;
    void* report_  = nullptr;
    void* close_   = nullptr;
};

}

}
