#pragma once

#include <cerfcore/peripherals/peripheral_base.h>

#include <array>
#include <functional>

namespace cerfcore {

class Au1300UsbControl : public Peripheral {
public:
    using Peripheral::Peripheral;
    bool ShouldRegister() override;
    void OnReady() override;
    uint32_t MmioBase() const override { return 0x14021000u; }
    uint32_t MmioSize() const override { return 0xCCu; }
    uint32_t ReadWord(uint32_t address) override;
    void WriteWord(uint32_t address, uint32_t value) override;
    void SaveState(StateWriter& w) override;
    void RestoreState(StateReader& r) override;
    void PostRestore() override;
    void SetOhciInterrupt(uint32_t index, bool level);
    bool OhciClockEnabled(uint32_t index) const;
    void RegisterClockListener(uint32_t index, std::function<void(bool)> listener);
    void RegisterEhciClockListener(std::function<void(bool)> listener);
    void SetEhciInterrupt(bool level);
    void RegisterUdcClockListener(std::function<void(bool)> listener);
    void SetUdcInterrupt(bool level);

private:
    void Publish();
    std::array<uint32_t, 14u> registers_{};
    uint32_t ohci_levels_ = 0u;
    std::array<std::function<void(bool)>, 2u> clock_listeners_;
    std::function<void(bool)> ehci_clock_listener_;
    std::function<void(bool)> udc_clock_listener_;
};

}
