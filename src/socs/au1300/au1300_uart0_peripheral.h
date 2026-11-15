#pragma once

#include "au1300_peripheral.h"

namespace cerfcore {

class Au1300Uart0Peripheral final : public Au1300Peripheral<Au1300UartModel, 0x10100000u, 0x1000u, 0u> {
public:
    using Au1300Peripheral::Au1300Peripheral;
};

}
