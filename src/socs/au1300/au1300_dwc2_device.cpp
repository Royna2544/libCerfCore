#include "au1300_id.h"
#include "au1300_usb_control.h"
#include "../../peripherals/peripheral_dispatcher.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/peripherals/peripheral_base.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>

#include <array>

namespace cerfcore {
namespace {

class Au1300Dwc2Device : public Peripheral {
public:
    explicit Au1300Dwc2Device(CerfEmulator& emu) : Peripheral(emu) { Reset(false); }
    bool ShouldRegister() override { return emu_.Get<BoardContext>().GetSocId() == SocId::Au1300; }
    uint32_t MmioBase() const override { return 0x14022000u; }
    uint32_t MmioSize() const override { return 0x11000u; }
    void OnReady() override {
        control_ = &emu_.Get<Au1300UsbControl>();
        control_->RegisterUdcClockListener([this](bool enabled) {
            clock_enabled_ = enabled;
            Publish();
        });
        emu_.Get<PeripheralDispatcher>().Register(this);
        auto& reset = emu_.Get<GuestCpuReset>();
        reset.RegisterResetListener([this](ResetLineKind) { Reset(false); });
        reset.RegisterResetReleaseListener([this] { Publish(); });
    }
    uint32_t ReadWord(uint32_t address) override {
        const auto offset = address - MmioBase();
        switch (offset) {
        case 0u: return global_[0u] | 0x10000u;
        case 4u: case 8u: case 0xCu: case 0x18u: case 0x24u: case 0x28u:
            return global_[offset / 4u];
        case 0x10u: return 0x80000000u;
        case 0x14u: return InterruptStatus();
        case 0x800u: return device_[0u];
        case 0x804u: return device_[1u];
        case 0x808u: return 1u;
        case 0x810u: case 0x814u: case 0x81Cu: return device_[(offset - 0x800u) / 4u];
        case 0x818u: return 0u;
        default: HaltUnsupportedAccess("DWC2 ReadWord", address, 0u);
        }
    }
    void WriteWord(uint32_t address, uint32_t value) override {
        const auto offset = address - MmioBase();
        switch (offset) {
        case 0u:
            if ((value & ~2u) != 0u) HaltUnsupportedAccess("DWC2 OTG control", address, value);
            global_[0u] = value;
            break;
        case 4u: global_[1u] &= ~value; break;
        case 8u: global_[2u] = value & 0x1BFu; break;
        case 0xCu:
            if ((value & (1u << 29u)) != 0u) HaltUnsupportedAccess("DWC2 host mode", address, value);
            global_[3u] = value & 0x5FFFFFFFu;
            break;
        case 0x10u:
            if ((value & ~0x800007FFu) != 0u) HaltUnsupportedAccess("DWC2 reset/flush", address, value);
            if ((value & 1u) != 0u) Reset(true);
            break;
        case 0x14u: global_[5u] &= ~value; break;
        case 0x18u: global_[6u] = value; break;
        case 0x24u: global_[9u] = value & 0xFFFFu; break;
        case 0x28u: global_[10u] = value; break;
        case 0x800u: device_[0u] = value & 0x00FE1FF7u; break;
        case 0x804u: WriteControl(address, value); break;
        case 0x810u: device_[4u] = value & 0x3FFu; break;
        case 0x814u: device_[5u] = value & 0x3FFu; break;
        case 0x81Cu: device_[7u] = value; break;
        default: HaltUnsupportedAccess("DWC2 WriteWord", address, value);
        }
        Publish();
    }
    void SaveState(StateWriter& w) override {
        w.Write("global", global_);
        w.Write("device", device_);
        w.Write("clock_enabled", clock_enabled_);
    }
    void RestoreState(StateReader& r) override {
        r.Read("global", global_);
        r.Read("device", device_);
        r.Read("clock_enabled", clock_enabled_);
        if ((global_[0u] & ~2u) != 0u || (device_[1u] & ~0x87Fu) != 0u) {
            r.Reject("Au1300 DWC2 register state");
        }
    }
    void PostRestore() override { Publish(); }

private:
    void Reset(bool preserve_usb_config) {
        const auto usb_config = preserve_usb_config ? global_[3u] : 0u;
        global_ = {};
        device_ = {};
        global_[3u] = usb_config;
        global_[9u] = 256u;
        global_[10u] = 0x01000100u;
    }
    void WriteControl(uint32_t address, uint32_t value) {
        if ((value & ~0xFFFu) != 0u || (value & 1u) != 0u) {
            HaltUnsupportedAccess("DWC2 device control", address, value);
        }
        auto state = (value & 0x873u) | (device_[1u] & 0xCu);
        if ((value & 0x80u) != 0u) state |= 4u;
        if ((value & 0x100u) != 0u) state &= ~4u;
        if ((value & 0x200u) != 0u) state |= 8u;
        if ((value & 0x400u) != 0u) state &= ~8u;
        device_[1u] = state;
    }
    uint32_t InterruptStatus() const {
        uint32_t status = global_[5u] | 0x20u;
        if ((device_[1u] & 4u) != 0u) status |= 0x40u;
        if ((device_[1u] & 8u) != 0u) status |= 0x80u;
        return status;
    }
    void Publish() {
        control_->SetUdcInterrupt(clock_enabled_ && (global_[2u] & 1u) != 0u &&
                                  (InterruptStatus() & global_[6u]) != 0u);
    }
    Au1300UsbControl* control_ = nullptr;
    std::array<uint32_t, 11u> global_{};
    std::array<uint32_t, 8u> device_{};
    bool clock_enabled_ = false;
};

}
REGISTER_SERVICE(Au1300Dwc2Device);
}
