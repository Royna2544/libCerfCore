#include "au1300_model_host.h"

#include "au1300_id.h"
#include "au1300_clock_peripheral.h"
#include "au1300_dma_peripheral.h"
#include "au1300_gpic_peripheral.h"
#include "au1300_uart0_peripheral.h"
#include "au1300_uart1_peripheral.h"
#include "au1300_uart2_peripheral.h"
#include "au1300_uart3_peripheral.h"
#include "../../execution/mips_engine_bridge.h"
#include "../../peripherals/peripheral_dispatcher.h"
#include "../../tracing/kernel_debug_sink.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/cpu/emulated_memory.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfengine/byte_order.h>
#include <cerfengine/mips/mips_bus_master_coherence.h>
#include <cerfengine/mips/mips_cpu.h>
#include <cerfengine/mips/mips_interrupt_channel.h>

#include <cstdarg>
#include <cstdio>
#include <string_view>

namespace cerfcore {

namespace {
constexpr std::array<std::string_view, 4u> kUartNames = {"UART0", "UART1", "UART2", "UART3"};
}

REGISTER_SERVICE(Au1300ModelHost);

bool Au1300ModelHost::ShouldRegister() {
    return emu_.Get<BoardContext>().GetSocId() == SocId::Au1300;
}

void Au1300ModelHost::OnReady() {
    auto& engine = emu_.Get<MipsEngineBridge>().Engine();
    state_ = engine.Get<cerfengine::MipsCpu>().State();
    interrupts_ = &engine.Get<cerfengine::MipsInterruptChannel>();
    coherence_ = &engine.Get<cerfengine::MipsBusMasterCoherence>();
    memory_ = &emu_.Get<EmulatedMemory>();
    peripherals_ = &emu_.Get<PeripheralDispatcher>();
    debug_ = &emu_.Get<KernelDebugSink>();
    clock_ = &emu_.Get<GuestCycleClock>();
    sys_event_ = clock_->Add([this] { Sys().Tick(); });
    dma_event_ = clock_->Add([this] { Dma().Tick(); });
    for (uint32_t i = 0u; i < uart_events_.size(); ++i) {
        uart_events_[i] = clock_->Add([this, i] { Uart(i).Advance(); });
    }
    emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) { OnShutdown(); });
}

void Au1300ModelHost::OnShutdown() {
    if (!clock_) return;
    if (sys_event_) clock_->Disarm(sys_event_);
    if (dma_event_) clock_->Disarm(dma_event_);
    for (auto* event : uart_events_) if (event) clock_->Disarm(event);
}

uint64_t Au1300ModelHost::Cycles() { return clock_->Cycles(); }
uint32_t Au1300ModelHost::Pc() const { return state_->pc; }
void Au1300ModelHost::SetCpuHz(uint64_t hz) { clock_->SetClockHz(hz); }

void Au1300ModelHost::Arm(GuestCycleClock::Event* event, uint64_t cycle) {
    if (cycle == UINT64_MAX) clock_->Disarm(event);
    else clock_->Arm(event, cycle);
}

void Au1300ModelHost::SetSysDeadline(uint64_t cycle) { Arm(sys_event_, cycle); }
void Au1300ModelHost::SetDmaDeadline(uint64_t cycle) { Arm(dma_event_, cycle); }
void Au1300ModelHost::SetUartDeadline(uint32_t index, uint64_t cycle) {
    Arm(uart_events_.at(index), cycle);
}
void Au1300ModelHost::SetGpicLines(uint32_t lines) {
    interrupts_->SetExternalInterruptLevel(lines);
}

uint32_t Au1300ModelHost::ReadPhysical(uint32_t pa, uint32_t width) {
    if (width == 0u || width > 4u || static_cast<uint64_t>(pa) + width > 0x100000000ull) {
        Fail("Au1300 DMA read pa=%08X width=%u", pa, width);
    }
    if (const auto* bytes = memory_->TryTranslateRange(pa, width)) {
        return static_cast<uint32_t>(cerfengine::le::UN(bytes, width));
    }
    if (width == 3u) Fail("Au1300 DMA MMIO read pa=%08X width=%u", pa, width);
    return peripherals_->Read(pa, static_cast<MmioWidth>(width));
}

void Au1300ModelHost::WritePhysical(uint32_t pa, uint32_t value, uint32_t width) {
    if (width == 0u || width > 4u || static_cast<uint64_t>(pa) + width > 0x100000000ull) {
        Fail("Au1300 DMA write pa=%08X width=%u", pa, width);
    }
    if (auto* bytes = memory_->TryTranslateRange(pa, width, true)) {
        cerfengine::le::PutN(bytes, value, width);
        coherence_->AfterWrite(pa, width);
        return;
    }
    if (width == 3u) Fail("Au1300 DMA MMIO write pa=%08X width=%u", pa, width);
    peripherals_->Write(pa, value, static_cast<MmioWidth>(width));
}

void Au1300ModelHost::EmitUartChar(uint32_t index, uint8_t value, std::string& line) {
    debug_->EmitChar(static_cast<char>(value), line, kUartNames.at(index));
}

void Au1300ModelHost::EmitUartLine(uint32_t index, const std::string& line) {
    debug_->EmitLine(line, kUartNames.at(index));
}

Au1300ClockModel& Au1300ModelHost::Sys() { return emu_.Get<Au1300ClockPeripheral>().Registers(); }
Au1300DmaModel& Au1300ModelHost::Dma() { return emu_.Get<Au1300DmaPeripheral>().Registers(); }
Au1300GpicModel& Au1300ModelHost::Gpic() { return emu_.Get<Au1300GpicPeripheral>().Registers(); }
Au1300UartModel& Au1300ModelHost::Uart(uint32_t index) {
    switch (index) {
    case 0u: return emu_.Get<Au1300Uart0Peripheral>().Registers();
    case 1u: return emu_.Get<Au1300Uart1Peripheral>().Registers();
    case 2u: return emu_.Get<Au1300Uart2Peripheral>().Registers();
    case 3u: return emu_.Get<Au1300Uart3Peripheral>().Registers();
    default: Fail("Au1300 UART index=%u", index);
    }
}

void Au1300ModelHost::Fail(const char* format, ...) const {
    char message[2048];
    va_list arguments;
    va_start(arguments, format);
    std::vsnprintf(message, sizeof message, format, arguments);
    va_end(arguments);
    emu_.Get<Fatal>().Die("%s", message);
}

}
