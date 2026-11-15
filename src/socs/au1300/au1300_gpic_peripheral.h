#pragma once

#include "au1300_peripheral.h"

namespace cerfcore {

class Au1300GpicPeripheral final : public Au1300Peripheral<Au1300GpicModel, 0x10200000u, 0x2000u, 0u> {
public:
    using Au1300Peripheral::Au1300Peripheral;
};

}
