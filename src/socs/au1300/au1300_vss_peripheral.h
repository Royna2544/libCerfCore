#pragma once

#include <cerfcore/peripherals/peripheral_base.h>

#include <array>

namespace cerfcore {

class Au1300VssPeripheral : public Peripheral {
public:
    enum class Block : uint32_t { Mpe, Bsa, Gpe, Mgp };
    using Peripheral::Peripheral;
    bool ShouldRegister() override;
    void OnReady() override;
    uint32_t MmioBase() const override { return 0x11003000u; }
    uint32_t MmioSize() const override { return 0x30u; }
    uint32_t ReadWord(uint32_t address) override;
    void WriteWord(uint32_t address, uint32_t value) override;
    void SaveState(StateWriter& w) override;
    void RestoreState(StateReader& r) override;
    bool Enabled(Block block) const;

private:
    std::array<std::array<uint32_t, 3u>, 4u> registers_{};
};

}
