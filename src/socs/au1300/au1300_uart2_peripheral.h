#pragma once

#include "au1300_peripheral.h"

namespace cerfcore {

class Au1300Uart2Peripheral final : public Au1300Peripheral<Au1300UartModel, 0x10102000u, 0x1000u, 2u> {
public:
    using Au1300Peripheral::Au1300Peripheral;
};

}
