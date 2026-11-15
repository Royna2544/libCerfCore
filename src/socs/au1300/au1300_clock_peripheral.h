#pragma once

#include "au1300_peripheral.h"

namespace cerfcore {

class Au1300ClockPeripheral final : public Au1300Peripheral<Au1300ClockModel, 0x10900000u, 0x1000u, 0u> {
public:
    using Au1300Peripheral::Au1300Peripheral;
};

}
