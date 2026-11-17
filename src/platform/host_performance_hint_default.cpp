#include "host_performance_hint.h"

namespace cerfcore {

namespace host_platform {

HostPerformanceHint::~HostPerformanceHint() { Close(); }

bool HostPerformanceHint::OpenForCurrentThread(int64_t) { return false; }

void HostPerformanceHint::ReportWork(int64_t) {}

void HostPerformanceHint::Close() {
    session_ = nullptr;
    report_  = nullptr;
    close_   = nullptr;
}

}

}
