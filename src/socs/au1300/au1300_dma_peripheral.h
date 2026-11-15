#pragma once

#include "au1300_peripheral.h"

namespace cerfcore {

class Au1300DmaPeripheral final : public Au1300Peripheral<Au1300DmaModel, 0x14002000u, 0x1010u, 0u> {
public:
    using Au1300Peripheral::Au1300Peripheral;
};

}
