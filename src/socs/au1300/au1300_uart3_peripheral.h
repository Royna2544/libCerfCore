#pragma once

#include "au1300_peripheral.h"

namespace cerfcore {

class Au1300Uart3Peripheral final : public Au1300Peripheral<Au1300UartModel, 0x10103000u, 0x1000u, 3u> {
public:
    using Au1300Peripheral::Au1300Peripheral;
};

}
