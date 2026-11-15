#pragma once

#include <cerfcore/core/service.h>
#include "../../execution/guest_cycle_clock.h"

#include <array>
#include <cstdint>
#include <string>

namespace cerfengine {
class MipsBusMasterCoherence;
class MipsInterruptChannel;
struct MipsCpuState;
}

namespace cerfcore {

class Au1300ClockModel;
class Au1300DmaModel;
class Au1300GpicModel;
class Au1300UartModel;
class EmulatedMemory;
class KernelDebugSink;
class PeripheralDispatcher;

class Au1300ModelHost : public Service {
public:
    using Service::Service;
    bool ShouldRegister() override;
    void OnReady() override;
    void OnShutdown() override;

    uint64_t Cycles();
    uint32_t Pc() const;
    void SetCpuHz(uint64_t hz);
    void SetSysDeadline(uint64_t cycle);
    void SetDmaDeadline(uint64_t cycle);
    void SetUartDeadline(uint32_t index, uint64_t cycle);
    void SetGpicLines(uint32_t lines);
    uint32_t ReadPhysical(uint32_t pa, uint32_t width);
    void WritePhysical(uint32_t pa, uint32_t value, uint32_t width);
    void EmitUartChar(uint32_t index, uint8_t value, std::string& line);
    void EmitUartLine(uint32_t index, const std::string& line);
    Au1300ClockModel& Sys();
    Au1300DmaModel& Dma();
    Au1300GpicModel& Gpic();
    Au1300UartModel& Uart(uint32_t index);
    [[noreturn]] void Fail(const char* format, ...) const;

private:
    void Arm(GuestCycleClock::Event* event, uint64_t cycle);
    GuestCycleClock* clock_ = nullptr;
    GuestCycleClock::Event* sys_event_ = nullptr;
    GuestCycleClock::Event* dma_event_ = nullptr;
    std::array<GuestCycleClock::Event*, 4u> uart_events_{};
    cerfengine::MipsCpuState* state_ = nullptr;
    cerfengine::MipsInterruptChannel* interrupts_ = nullptr;
    cerfengine::MipsBusMasterCoherence* coherence_ = nullptr;
    EmulatedMemory* memory_ = nullptr;
    PeripheralDispatcher* peripherals_ = nullptr;
    KernelDebugSink* debug_ = nullptr;
};

}
