#include "au1300_usb_control.h"

#include "au1300_id.h"
#include "au1300_gpic_model.h"
#include "au1300_model_host.h"
#include "../../peripherals/peripheral_dispatcher.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/state/state_stream.h>
#include <cerfcore/socs/guest_cpu_reset.h>

namespace cerfcore {

REGISTER_SERVICE(Au1300UsbControl);

bool Au1300UsbControl::ShouldRegister() {
    return emu_.Get<BoardContext>().GetSocId() == SocId::Au1300;
}

void Au1300UsbControl::OnReady() {
    emu_.Get<PeripheralDispatcher>().Register(this);
    auto& reset = emu_.Get<GuestCpuReset>();
    reset.RegisterResetListener([this](ResetLineKind) {
        registers_ = {};
        ohci_levels_ = 0u;
    });
    reset.RegisterResetReleaseListener([this] {
        for (auto& listener : clock_listeners_) listener(false);
        ehci_clock_listener_(false);
        udc_clock_listener_(false);
        Publish();
    });
}

uint32_t Au1300UsbControl::ReadWord(uint32_t address) {
    const uint32_t offset = address - MmioBase();
    switch (offset) {
    case 0u: case 4u: case 0x10u: case 0x14u: case 0x18u:
    case 0x1Cu: case 0x20u: case 0x28u: case 0x2Cu: case 0x30u: case 0x34u:
        return registers_[offset / 4u];
    case 0xC4u: return ohci_levels_;
    case 0xC8u: return registers_[2u];
    default: HaltUnsupportedAccess("ReadWord", address, 0u);
    }
}

void Au1300UsbControl::WriteWord(uint32_t address, uint32_t value) {
    const uint32_t offset = address - MmioBase();
    uint32_t mask = 0u;
    switch (offset) {
    case 0u: mask = 7u; break;
    case 4u: mask = 15u; break;
    case 0x14u: mask = 4u; break;
    case 0x18u:
        registers_[offset / 4u] &= ~value;
        return;
    case 0x1Cu: mask = 0xF0003u; break;
    case 0x20u: mask = 0x10004u; break;
    case 0x28u: mask = 0x10u; break;
    case 0x2Cu: break;
    case 0x34u: mask = 1u; break;
    case 0xC4u:
        Publish();
        return;
    case 0xC8u:
        if ((value & ~15u) != 0u) HaltUnsupportedAccess("USB interrupt enable", address, value);
        registers_[2u] = value;
        Publish();
        return;
    default: HaltUnsupportedAccess("WriteWord", address, value);
    }
    if ((value & ~mask) != 0u) HaltUnsupportedAccess("WriteWord", address, value);
    registers_[offset / 4u] = value;
    if (offset == 4u || offset == 0x1Cu) {
        for (uint32_t i = 0u; i < clock_listeners_.size(); ++i) {
            clock_listeners_[i](OhciClockEnabled(i));
        }
    }
    if (offset == 0u || offset == 4u || offset == 0x1Cu) {
        ehci_clock_listener_((registers_[0u] & 2u) != 0u && (registers_[1u] & 7u) == 7u &&
                             (registers_[0x1Cu / 4u] & 0x20000u) != 0u);
        udc_clock_listener_((registers_[0u] & 1u) != 0u && (registers_[1u] & 7u) == 7u &&
                            (registers_[0x1Cu / 4u] & 0x10000u) != 0u);
    }
}

void Au1300UsbControl::RegisterClockListener(uint32_t index, std::function<void(bool)> listener) {
    clock_listeners_.at(index) = std::move(listener);
}

void Au1300UsbControl::RegisterEhciClockListener(std::function<void(bool)> listener) {
    ehci_clock_listener_ = std::move(listener);
}

void Au1300UsbControl::SetEhciInterrupt(bool level) { SetOhciInterrupt(2u, level); }

void Au1300UsbControl::RegisterUdcClockListener(std::function<void(bool)> listener) {
    udc_clock_listener_ = std::move(listener);
}

void Au1300UsbControl::SetUdcInterrupt(bool level) { SetOhciInterrupt(3u, level); }

bool Au1300UsbControl::OhciClockEnabled(uint32_t index) const {
    return (registers_[0x1Cu / 4u] & (1u << (18u + index))) != 0u &&
           (registers_[1u] & 7u) == 7u;
}

void Au1300UsbControl::SetOhciInterrupt(uint32_t index, bool level) {
    const uint32_t bit = 1u << index;
    ohci_levels_ = level ? ohci_levels_ | bit : ohci_levels_ & ~bit;
    Publish();
}

void Au1300UsbControl::Publish() {
    emu_.Get<Au1300ModelHost>().Gpic().SetLevel(90u, (ohci_levels_ & registers_[2u]) != 0u);
}

void Au1300UsbControl::SaveState(StateWriter& w) {
    for (auto value : registers_) w.Write("register", value);
    w.Write("ohci_levels", ohci_levels_);
}

void Au1300UsbControl::RestoreState(StateReader& r) {
    for (auto& value : registers_) r.Read("register", value);
    r.Read("ohci_levels", ohci_levels_);
    if ((ohci_levels_ & ~15u) != 0u || (registers_[2u] & ~15u) != 0u) {
        r.Reject("Au1300 USB interrupt state");
    }
}

void Au1300UsbControl::PostRestore() { Publish(); }

}
