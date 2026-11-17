#pragma once

#include "../../socs/i2c_slave.h"

#include <cstdint>

namespace cerfcore {

class GuestCycleClock;

class Bh1750LightSensor : public I2cSlave {
public:
    using I2cSlave::I2cSlave;

    static constexpr uint8_t kAddress = 0x23u;

    bool ShouldRegister() override;
    void OnReady() override;

    bool MatchesAddress(uint8_t slave_addr) const override { return slave_addr == kAddress; }
    void TxnStart(uint8_t slave_addr) override;
    void TxnWriteByte(uint8_t slave_addr, uint8_t byte) override;
    uint8_t TxnReadByte(uint8_t slave_addr) override;

    void SaveState(StateWriter& w) override;
    void RestoreState(StateReader& r) override;

private:
    GuestCycleClock* clock_ = nullptr;
    int64_t          ready_ns_ = 0;
    uint16_t         sample_ = 0u;
    uint8_t          mode_ = 0u;
    uint8_t          read_index_ = 0u;
    bool             powered_ = false;
};

}
