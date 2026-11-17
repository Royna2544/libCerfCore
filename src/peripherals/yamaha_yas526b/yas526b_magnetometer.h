#pragma once

#include "../../socs/i2c_slave.h"

#include <array>
#include <cstdint>

namespace cerfcore {

class HostMotionFeed;

class Yas526bMagnetometer : public I2cSlave {
public:
    using I2cSlave::I2cSlave;

    static constexpr uint8_t kXyAddress = 0x2Eu;
    static constexpr uint8_t kZAddress  = 0x2Fu;

    bool ShouldRegister() override;
    void OnReady() override;

    bool MatchesAddress(uint8_t slave_addr) const override {
        return slave_addr == kXyAddress || slave_addr == kZAddress;
    }
    void    TxnStart(uint8_t slave_addr) override;
    void    TxnWriteByte(uint8_t slave_addr, uint8_t byte) override;
    uint8_t TxnReadByte(uint8_t slave_addr) override;

    void SaveState(StateWriter& w) override;
    void RestoreState(StateReader& r) override;

private:
    struct Part {
        std::array<uint8_t, 4> result{};
        std::array<uint8_t, 2> offset{};
        uint8_t                config = 0u;
        uint8_t                read_index = 0u;
    };

    Part& PartFor(uint8_t slave_addr);
    void  Measure(uint8_t slave_addr, uint8_t mode);
    static void Pack(uint16_t value, uint8_t* out);

    HostMotionFeed* feed_ = nullptr;
    Part            xy_;
    Part            z_;
};

}
