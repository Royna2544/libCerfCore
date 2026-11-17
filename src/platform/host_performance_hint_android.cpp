#include "host_performance_hint.h"

#include <dlfcn.h>
#include <sys/types.h>
#include <unistd.h>

namespace cerfcore {

namespace host_platform {

namespace {

using GetManager    = void* (*)();
using CreateSession = void* (*)(void*, const int32_t*, size_t, int64_t);
using ReportActual  = int (*)(void*, int64_t);
using CloseSession  = void (*)(void*);

}

HostPerformanceHint::~HostPerformanceHint() { Close(); }

bool HostPerformanceHint::OpenForCurrentThread(int64_t target_ns) {
    Close();
    void* const library = ::dlopen("libandroid.so", RTLD_NOW | RTLD_NOLOAD);
    if (library == nullptr) return false;
    const auto get_manager = reinterpret_cast<GetManager>(::dlsym(library, "APerformanceHint_getManager"));
    const auto create      = reinterpret_cast<CreateSession>(::dlsym(library, "APerformanceHint_createSession"));
    report_ = ::dlsym(library, "APerformanceHint_reportActualWorkDuration");
    close_  = ::dlsym(library, "APerformanceHint_closeSession");
    if (get_manager == nullptr || create == nullptr || report_ == nullptr || close_ == nullptr) return false;
    void* const manager = get_manager();
    if (manager == nullptr) return false;
    const int32_t thread = static_cast<int32_t>(::gettid());
    session_ = create(manager, &thread, 1u, target_ns);
    return session_ != nullptr;
}

void HostPerformanceHint::ReportWork(int64_t actual_ns) {
    if (session_ == nullptr || actual_ns <= 0) return;
    reinterpret_cast<ReportActual>(report_)(session_, actual_ns);
}

void HostPerformanceHint::Close() {
    if (session_ == nullptr) return;
    reinterpret_cast<CloseSession>(close_)(session_);
    session_ = nullptr;
}

}

}
