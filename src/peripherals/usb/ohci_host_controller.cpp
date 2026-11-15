#include "ohci_host_controller.h"

#include "../../peripherals/peripheral_dispatcher.h"
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/state/state_stream.h>
#include <cerfcore/socs/guest_cpu_reset.h>

namespace cerfcore {

OhciHostController::OhciHostController(CerfEmulator& emu) : Peripheral(emu), bits_(emu) {
    Reset(false);
}

void OhciHostController::OnReady() {
    clock_ = &emu_.Get<GuestCycleClock>();
    bits_.SetRate(clock_->ClockRate(), {12000000u, 1u});
    bits_.Anchor(clock_->Cycles(), 0u);
    event_ = clock_->Add([this] { Advance(); Arm(); });
    clock_->RegisterRateListener([this] {
        Advance();
        bits_.RescaleRate(clock_->Cycles(), clock_->ClockRate(), {12000000u, 1u});
        Arm();
    });
    emu_.Get<PeripheralDispatcher>().Register(this);
    auto& reset = emu_.Get<GuestCpuReset>();
    reset.RegisterResetListener([this](ResetLineKind) {
        Reset(false);
        clock_->Disarm(event_);
    });
    reset.RegisterResetReleaseListener([this] { Publish(); Arm(); });
}

void OhciHostController::OnShutdown() {
    if (clock_ && event_) clock_->Disarm(event_);
}

void OhciHostController::Reset(bool software) {
    const auto hub = registers_[18u];
    const auto descriptor = registers_[19u];
    const auto status = registers_[20u];
    const auto port = registers_[21u];
    const auto routing = registers_[1u] & 0x300u;
    registers_.fill(0u);
    registers_[1u] = software ? routing | 0xC0u : 0u;
    registers_[13u] = 0x2EDFu;
    registers_[17u] = 0x628u;
    registers_[18u] = software ? hub : 0x01000201u;
    registers_[19u] = software ? descriptor : 0u;
    registers_[20u] = software ? status : 0u;
    registers_[21u] = software ? port : 0x100u;
    frame_left_ = 0u;
    frame_toggle_ = 0u;
    periodic_done_ = false;
}

uint32_t OhciHostController::ReadWord(uint32_t address) {
    Advance();
    const uint32_t offset = address - MmioBase();
    if (offset == 0u) return 0x10u;
    if (offset == 0x14u) return registers_[4u];
    if (offset == 0x38u) return (frame_left_ == 0u ? 0u : frame_left_ - 1u) | frame_toggle_;
    if (offset > 0x54u || (offset & 3u) != 0u) HaltUnsupportedAccess("ReadWord", address, 0u);
    return registers_[offset / 4u];
}

void OhciHostController::WriteWord(uint32_t address, uint32_t value) {
    Advance();
    const uint32_t offset = address - MmioBase();
    switch (offset) {
    case 4u: {
        const bool was_operational = Operational();
        registers_[1u] = value & 0x7FFu;
        if (!was_operational && Operational()) {
            frame_left_ = (registers_[13u] & 0x3FFFu) + 1u;
            frame_toggle_ = registers_[13u] & 0x80000000u;
            periodic_done_ = true;
        }
        break;
    }
    case 8u:
        registers_[2u] |= value & 6u;
        if ((value & 1u) != 0u) Reset(true);
        if ((value & 8u) != 0u) registers_[3u] |= 0x40000000u;
        break;
    case 0xCu: registers_[3u] &= ~(value & 0x4000007Fu); break;
    case 0x10u: registers_[4u] |= value & 0xC000007Fu; break;
    case 0x14u: registers_[4u] &= ~(value & 0xC000007Fu); break;
    case 0x18u: registers_[6u] = value & 0xFFFFFF00u; break;
    case 0x1Cu: case 0x20u: case 0x24u: case 0x28u: case 0x2Cu:
        registers_[offset / 4u] = value & 0xFFFFFFF0u;
        break;
    case 0x34u: registers_[13u] = value & 0xFFFF3FFFu; break;
    case 0x40u: registers_[16u] = value & 0x3FFFu; break;
    case 0x44u: registers_[17u] = value & 0xFFFu; break;
    case 0x48u: registers_[18u] = (value & 0xFF001B00u) | 1u; break;
    case 0x4Cu: registers_[19u] = value & 0x00020002u; break;
    case 0x50u: WriteHub(value); break;
    case 0x54u: WritePort(value); break;
    default: HaltUnsupportedAccess("WriteWord", address, value);
    }
    Publish();
    Arm();
}

void OhciHostController::Publish() {
    const bool enabled = (registers_[4u] & 0x80000000u) != 0u;
    const bool routed = (registers_[1u] & 0x100u) != 0u;
    SetInterrupt(enabled && !routed && (registers_[3u] & registers_[4u] & 0x7Fu) != 0u);
}

void OhciHostController::Power(bool on) {
    auto& status = registers_[21u];
    if ((registers_[18u] & 0x200u) != 0u) on = true;
    if (on) status |= 0x100u;
    else status &= ~0x117u;
    const bool connected = on && port_.IsConnected();
    if (connected != ((status & 1u) != 0u)) {
        status = (status & ~1u) | (connected ? 1u : 0u) | 0x10000u;
        registers_[3u] |= 0x40u;
    }
}

void OhciHostController::WriteHub(uint32_t value) {
    if ((value & 0x80000000u) != 0u) registers_[20u] &= ~0x8000u;
    if ((value & 0x8000u) != 0u) registers_[20u] |= 0x8000u;
    registers_[20u] &= ~(value & 0x20000u);
    const bool global = (registers_[18u] & 0x100u) == 0u || (registers_[19u] & 0x20000u) == 0u;
    if (global && (value & 1u) != 0u) Power(false);
    if (global && (value & 0x10000u) != 0u) Power(true);
}

void OhciHostController::WritePort(uint32_t value) {
    auto& status = registers_[21u];
    status &= ~(value & 0x1F0000u);
    if ((value & 1u) != 0u) status &= ~2u;
    if ((value & 0x16u) != 0u) {
        if ((status & 1u) == 0u) { status |= 0x10000u; registers_[3u] |= 0x40u; }
        else HaltUnsupportedAccess("connected OHCI port operation", MmioBase() + 0x54u, value);
    }
    const bool individual = (registers_[18u] & 0x100u) != 0u && (registers_[19u] & 0x20000u) != 0u;
    if (individual && (value & 0x200u) != 0u) Power(false);
    if (individual && (value & 0x100u) != 0u) Power(true);
}

void OhciHostController::OnPortConnectChanged(int) {
    Advance();
    Power((registers_[21u] & 0x100u) != 0u);
    Publish();
}

void OhciHostController::SaveState(StateWriter& w) {
    Advance();
    w.Write("registers", registers_);
    w.Write("last_bits", last_bits_);
    w.Write("frame_left", frame_left_);
    w.Write("frame_toggle", frame_toggle_);
    w.Write("periodic_done", periodic_done_);
    w.Write("clocked", clocked_);
    const auto count = bits_.CountAt(clock_->Cycles());
    w.Write("bit_count", count);
    bits_.TickPhaseAt(clock_->Cycles()).Save(w, "bit_phase");
    port_.SaveState(w);
}

void OhciHostController::RestoreState(StateReader& r) {
    r.Read("registers", registers_);
    r.Read("last_bits", last_bits_);
    r.Read("frame_left", frame_left_);
    r.Read("frame_toggle", frame_toggle_);
    r.Read("periodic_done", periodic_done_);
    r.Read("clocked", clocked_);
    r.Read("bit_count", restored_bits_);
    restored_phase_.Restore(r, "bit_phase");
    port_.RestoreState(r);
    if ((registers_[18u] & 0xFFu) != 1u || frame_left_ > 0x4000u) r.Reject("OHCI frame/port state");
    bits_.SetRate(clock_->ClockRate(), {12000000u, 1u});
    bits_.AnchorAt(clock_->Cycles(), restored_bits_, restored_phase_);
}

void OhciHostController::PostRestore() {
    bits_.SetRate(clock_->ClockRate(), {12000000u, 1u});
    bits_.AnchorAt(clock_->Cycles(), restored_bits_, restored_phase_);
    port_.PostRestore();
    Publish();
    Arm();
}

}
