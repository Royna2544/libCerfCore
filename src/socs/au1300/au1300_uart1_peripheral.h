#pragma once

#include "au1300_peripheral.h"

namespace cerfcore {

class Au1300Uart1Peripheral final : public Au1300Peripheral<Au1300UartModel, 0x10101000u, 0x1000u, 1u> {
public:
    using Au1300Peripheral::Au1300Peripheral;
};

}
