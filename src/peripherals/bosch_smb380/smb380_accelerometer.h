#pragma once

#include "../../socs/i2c_slave.h"

#include <array>
#include <cstdint>

namespace cerfcore {

class HostMotionFeed;

class Smb380Accelerometer : public I2cSlave {
public:
    using I2cSlave::I2cSlave;

    static constexpr uint8_t kAddress = 0x38u;

    bool ShouldRegister() override;
    void OnReady() override;

    bool    MatchesAddress(uint8_t slave_addr) const override { return slave_addr == kAddress; }
    void    TxnStart(uint8_t slave_addr) override;
    void    TxnWriteByte(uint8_t slave_addr, uint8_t byte) override;
    uint8_t TxnReadByte(uint8_t slave_addr) override;

    void SaveState(StateWriter& w) override;
    void RestoreState(StateReader& r) override;

private:
    static constexpr uint8_t kRegisterCount = 0x16u;

    void Reset();
    void LatchSample();
    void WriteRegister(uint8_t reg, uint8_t value);

    HostMotionFeed*                       feed_ = nullptr;
    std::array<uint8_t, kRegisterCount>   regs_{};
    std::array<uint8_t, 6>                sample_{};
    uint8_t                               pointer_     = 0u;
    bool                                  have_pointer_ = false;
};

}
