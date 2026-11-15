#pragma once

#include "au1300_ohci_peripheral.h"

namespace cerfcore {

class Au1300Ohci0Peripheral final : public Au1300OhciPeripheral<0u> {
public:
    using Au1300OhciPeripheral::Au1300OhciPeripheral;
};

}
