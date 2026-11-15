#include "au1300_ehci_peripheral.h"

#include "au1300_id.h"
#include "au1300_model_host.h"
#include "au1300_usb_control.h"
#include "../../peripherals/peripheral_dispatcher.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/state/state_stream.h>
#include <cerfcore/socs/guest_cpu_reset.h>

namespace cerfcore {

REGISTER_SERVICE(Au1300EhciPeripheral);

Au1300EhciPeripheral::Au1300EhciPeripheral(CerfEmulator& emu) : Peripheral(emu), uframes_(emu) {
    Reset();
}

bool Au1300EhciPeripheral::ShouldRegister() {
    return emu_.Get<BoardContext>().GetSocId() == SocId::Au1300;
}

void Au1300EhciPeripheral::OnReady() {
    clock_ = &emu_.Get<GuestCycleClock>();
    uframes_.SetRate(clock_->ClockRate(), {8000u, 1u});
    uframes_.Anchor(clock_->Cycles(), 0u);
    event_ = clock_->Add([this] { Advance(); Arm(); });
    clock_->RegisterRateListener([this] {
        Advance();
        uframes_.RescaleRate(clock_->Cycles(), clock_->ClockRate(), {8000u, 1u});
        Arm();
    });
    emu_.Get<Au1300UsbControl>().RegisterEhciClockListener([this](bool on) {
        Advance();
        clocked_ = on;
        Arm();
    });
    emu_.Get<PeripheralDispatcher>().Register(this);
    auto& reset = emu_.Get<GuestCpuReset>();
    reset.RegisterResetListener([this](ResetLineKind) {
        Reset();
        clock_->Disarm(event_);
    });
    reset.RegisterResetReleaseListener([this] { Publish(); Arm(); });
}

void Au1300EhciPeripheral::OnShutdown() {
    if (clock_ && event_) clock_->Disarm(event_);
}

void Au1300EhciPeripheral::Reset() {
    registers_.fill(0u);
    registers_[0u] = 0x80000u;
    registers_[1u] = 0x1000u;
    registers_[17u] = registers_[18u] = 0x2000u;
    reported_ = 0u;
}

uint32_t Au1300EhciPeripheral::Capability(uint32_t offset) const {
    switch (offset) {
    case 0x200u: return 0x01000020u;
    case 0x204u: return 0x2112u;
    case 0x208u: case 0x20Cu: case 0x210u: return 0u;
    default: HaltUnsupportedAccess("Capability", MmioBase() + offset, 0u);
    }
}

uint8_t Au1300EhciPeripheral::ReadByte(uint32_t address) {
    if (address < MmioBase() + 0x200u || address >= MmioBase() + 0x214u) {
        HaltUnsupportedAccess("ReadByte", address, 0u);
    }
    return static_cast<uint8_t>(Capability((address - MmioBase()) & ~3u) >> ((address & 3u) * 8u));
}

uint16_t Au1300EhciPeripheral::ReadHalf(uint32_t address) {
    if ((address & 1u) != 0u) HaltUnsupportedAccess("ReadHalf", address, 0u);
    return static_cast<uint16_t>(ReadByte(address) | static_cast<uint32_t>(ReadByte(address + 1u)) << 8u);
}

uint32_t Au1300EhciPeripheral::ReadWord(uint32_t address) {
    const auto offset = address - MmioBase();
    if (offset >= 0x200u && offset < 0x214u && (offset & 3u) == 0u) return Capability(offset);
    Advance();
    if (offset < 0x220u || offset > 0x268u || (offset & 3u) != 0u) {
        HaltUnsupportedAccess("ReadWord", address, 0u);
    }
    return registers_[(offset - 0x220u) / 4u];
}

void Au1300EhciPeripheral::WriteWord(uint32_t address, uint32_t value) {
    Advance();
    const uint32_t offset = address - MmioBase() - 0x220u;
    switch (offset) {
    case 0u:
        if ((value & 2u) != 0u) Reset();
        else {
            registers_[0u] = value & 0xFF0071u;
            if (!Running()) registers_[1u] = (registers_[1u] & ~0xC000u) | 0x1000u;
            else registers_[1u] &= ~0x1000u;
        }
        break;
    case 4u:
        registers_[1u] &= ~(value & 0x3Fu);
        reported_ &= registers_[1u];
        break;
    case 8u: registers_[2u] = value & 0x3Fu; break;
    case 0xCu:
        if (Running()) HaltUnsupportedAccess("running FRINDEX write", address, value);
        registers_[3u] = value & 0x3FFFu;
        break;
    case 0x10u: break;
    case 0x14u: registers_[5u] = value & 0xFFFFF000u; break;
    case 0x18u: registers_[6u] = value & 0xFFFFFFE0u; break;
    case 0x40u:
        registers_[16u] = value & 1u;
        for (uint32_t i : {17u, 18u}) {
            if (registers_[16u] != 0u) registers_[i] &= ~0x2000u;
            else registers_[i] |= 0x2000u;
        }
        break;
    case 0x44u: case 0x48u: {
        auto& port = registers_[offset / 4u];
        if ((value & 0xF0000u) != 0u) HaltUnsupportedAccess("EHCI port test mode", address, value);
        port &= ~(value & 0x2Au);
        port = (port & ~0x7031C0u) | (value & 0x703100u);
        if ((port & 0x1000u) == 0u) port &= ~0x7011C0u;
        break;
    }
    default: HaltUnsupportedAccess("WriteWord", address, value);
    }
    Publish();
    Arm();
}

void Au1300EhciPeripheral::Publish() {
    const auto pending = registers_[1u] & registers_[2u];
    emu_.Get<Au1300UsbControl>().SetEhciInterrupt((pending & 0x1Cu) != 0u || (pending & reported_) != 0u);
}

void Au1300EhciPeripheral::Walk(uint32_t link, bool periodic) {
    const auto first = link & ~31u;
    auto& bus = emu_.Get<Au1300ModelHost>();
    for (uint32_t visited = 0u; (link & 1u) == 0u; ++visited) {
        const auto address = link & ~31u;
        if (address == 0u || visited == 4096u || (link & 6u) != 2u) {
            emu_.Get<Fatal>().Die("EHCI invalid %s schedule link=%08X", periodic ? "periodic" : "async", link);
        }
        const auto token = bus.ReadPhysical(address + 24u, 4u);
        const auto next_td = bus.ReadPhysical(address + 16u, 4u);
        if ((token & 0x40u) == 0u && ((token & 0x80u) != 0u ||
            ((next_td & 1u) == 0u && (bus.ReadPhysical((next_td & ~31u) + 8u, 4u) & 0x80u) != 0u))) {
            emu_.Get<Fatal>().Die("EHCI active QH=%08X qTD=%08X", address, next_td);
        }
        link = bus.ReadPhysical(address, 4u);
        if (!periodic && (link & ~31u) == first) return;
    }
}

void Au1300EhciPeripheral::Microframe() {
    const auto previous = registers_[3u];
    registers_[3u] = (previous + 1u) & 0x3FFFu;
    if (((previous ^ registers_[3u]) & 0x2000u) != 0u) registers_[1u] |= 8u;
    registers_[1u] &= ~0xC000u;
    if ((registers_[0u] & 0x10u) != 0u) {
        const auto frame = (registers_[3u] >> 3u) & 1023u;
        Walk(emu_.Get<Au1300ModelHost>().ReadPhysical(registers_[5u] + frame * 4u, 4u), true);
        registers_[1u] |= 0x4000u;
    }
    if ((registers_[0u] & 0x20u) != 0u) {
        Walk(registers_[6u] | 2u, false);
        registers_[1u] |= 0x8000u;
        if ((registers_[0u] & 0x40u) != 0u) {
            registers_[0u] &= ~0x40u;
            registers_[1u] |= 0x20u;
        }
    }
    const auto threshold = (registers_[0u] >> 16u) & 0xFFu;
    if (threshold == 0u || registers_[3u] % threshold == 0u) {
        reported_ |= registers_[1u] & registers_[2u] & 0x23u;
    }
    Publish();
}

void Au1300EhciPeripheral::Advance() {
    if (clock_->Cycles() < uframes_.AnchorCycle()) {
        emu_.Get<Fatal>().Die("EHCI counter anchor is after restored guest clock: anchor=%llu clock=%llu",
            static_cast<unsigned long long>(uframes_.AnchorCycle()),
            static_cast<unsigned long long>(clock_->Cycles()));
    }
    const auto now = uframes_.CountAt(clock_->Cycles());
    const auto elapsed = now - last_uframe_;
    last_uframe_ = now;
    if (clocked_ && Running()) for (uint32_t i = 0u; i < elapsed; ++i) Microframe();
}

void Au1300EhciPeripheral::Arm() {
    if (!clocked_ || !Running()) clock_->Disarm(event_);
    else clock_->Arm(event_, uframes_.CycleOfTick(uframes_.TicksSince(clock_->Cycles()) + 1u));
}

void Au1300EhciPeripheral::SaveState(StateWriter& w) {
    Advance();
    w.Write("registers", registers_);
    w.Write("last_uframe", last_uframe_);
    w.Write("reported", reported_);
    w.Write("clocked", clocked_);
    const auto count = uframes_.CountAt(clock_->Cycles());
    w.Write("uframe_count", count);
    uframes_.TickPhaseAt(clock_->Cycles()).Save(w, "uframe_phase");
}

void Au1300EhciPeripheral::RestoreState(StateReader& r) {
    r.Read("registers", registers_);
    r.Read("last_uframe", last_uframe_);
    r.Read("reported", reported_);
    r.Read("clocked", clocked_);
    r.Read("uframe_count", restored_uframe_);
    restored_phase_.Restore(r, "uframe_phase");
    if ((registers_[3u] & ~0x3FFFu) != 0u || (reported_ & ~0x23u) != 0u) r.Reject("EHCI frame/IRQ state");
    uframes_.SetRate(clock_->ClockRate(), {8000u, 1u});
    uframes_.AnchorAt(clock_->Cycles(), restored_uframe_, restored_phase_);
}

void Au1300EhciPeripheral::PostRestore() {
    uframes_.SetRate(clock_->ClockRate(), {8000u, 1u});
    uframes_.AnchorAt(clock_->Cycles(), restored_uframe_, restored_phase_);
    Publish();
    Arm();
}

}
