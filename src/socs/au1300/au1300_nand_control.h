#pragma once

#include <cerfcore/peripherals/peripheral_base.h>

namespace cerfcore {

class Au1300NandControl : public Peripheral {
public:
    using Peripheral::Peripheral;
    bool ShouldRegister() override;
    void OnReady() override;
    uint32_t MmioBase() const override;
    uint32_t MmioSize() const override;
    uint32_t ReadWord(uint32_t address) override;
    void WriteWord(uint32_t address, uint32_t value) override;
    void SaveState(StateWriter& w) override;
    void RestoreState(StateReader& r) override;
    void PostRestore() override;
    void EndAccess();

private:
    uint32_t control_ = 0u;
};

}
