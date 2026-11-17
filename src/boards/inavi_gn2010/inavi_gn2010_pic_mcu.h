#pragma once

#include "../../socs/i2c_slave.h"

#include <cstdint>

namespace cerfcore {

class InaviGn2010PicMcu : public I2cSlave {
public:
    using I2cSlave::I2cSlave;

    static constexpr uint8_t kAddress = 0x2Au;

    bool ShouldRegister() override;

    bool    MatchesAddress(uint8_t slave_addr) const override { return slave_addr == kAddress; }
    void    TxnStart(uint8_t slave_addr) override;
    void    TxnWriteByte(uint8_t slave_addr, uint8_t byte) override;
    uint8_t TxnReadByte(uint8_t slave_addr) override;
};

}
