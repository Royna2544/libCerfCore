#pragma once

#include "../cycle_anchored_counter.h"
#include <cerfcore/peripherals/peripheral_base.h>

#include <array>

namespace cerfcore {

class Au1300EhciPeripheral : public Peripheral {
public:
    explicit Au1300EhciPeripheral(CerfEmulator& emu);
    bool ShouldRegister() override;
    void OnReady() override;
    void OnShutdown() override;
    uint32_t MmioBase() const override { return 0x14020000u; }
    uint32_t MmioSize() const override { return 0x400u; }
    uint8_t ReadByte(uint32_t address) override;
    uint16_t ReadHalf(uint32_t address) override;
    uint32_t ReadWord(uint32_t address) override;
    void WriteWord(uint32_t address, uint32_t value) override;
    void SaveState(StateWriter& w) override;
    void RestoreState(StateReader& r) override;
    void PostRestore() override;

private:
    void Reset();
    void Advance();
    void Arm();
    void Publish();
    void Microframe();
    void Walk(uint32_t link, bool periodic);
    uint32_t Capability(uint32_t offset) const;
    bool Running() const { return (registers_[0u] & 1u) != 0u; }
    GuestCycleClock* clock_ = nullptr;
    GuestCycleClock::Event* event_ = nullptr;
    CycleAnchoredCounter uframes_;
    std::array<uint32_t, 19u> registers_{};
    uint32_t last_uframe_ = 0u;
    uint32_t reported_ = 0u;
    bool clocked_ = false;
    uint32_t restored_uframe_ = 0u;
    CounterPhase restored_phase_;
};

}
