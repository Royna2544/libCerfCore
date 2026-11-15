#pragma once

#include "usb_host_port.h"
#include "../../socs/cycle_anchored_counter.h"
#include <cerfcore/peripherals/peripheral_base.h>

#include <array>

namespace cerfcore {

class OhciHostController : public Peripheral, public UsbHostPortHost {
public:
    explicit OhciHostController(CerfEmulator& emu);
    void OnReady() override;
    void OnShutdown() override;
    uint32_t MmioSize() const override { return 0x100u; }
    uint32_t ReadWord(uint32_t address) override;
    void WriteWord(uint32_t address, uint32_t value) override;
    void SaveState(StateWriter& w) override;
    void RestoreState(StateReader& r) override;
    void PostRestore() override;
    void SetClocked(bool clocked);
    void OnPortConnectChanged(int port_index) override;
    UsbHostPort& Port() { return port_; }

protected:
    virtual uint32_t ReadBus(uint32_t pa, uint32_t width) = 0;
    virtual void WriteBus(uint32_t pa, uint32_t value, uint32_t width) = 0;
    virtual void SetInterrupt(bool level) = 0;

private:
    void Reset(bool software);
    void Advance();
    void Arm();
    void BeginFrame();
    void Periodic();
    void NonPeriodic(uint32_t head, uint32_t current, uint32_t filled, uint32_t enable);
    void Walk(uint32_t& current);
    void Publish();
    void WriteHub(uint32_t value);
    void WritePort(uint32_t value);
    void Power(bool on);
    bool Operational() const { return (registers_[1u] & 0xC0u) == 0x80u; }

    UsbHostPort port_{*this, 0};
    CycleAnchoredCounter bits_;
    GuestCycleClock* clock_ = nullptr;
    GuestCycleClock::Event* event_ = nullptr;
    std::array<uint32_t, 24u> registers_{};
    uint32_t last_bits_ = 0u;
    uint32_t frame_left_ = 0u;
    uint32_t frame_toggle_ = 0u;
    bool periodic_done_ = false;
    bool clocked_ = false;
    uint32_t restored_bits_ = 0u;
    CounterPhase restored_phase_;
};

}
