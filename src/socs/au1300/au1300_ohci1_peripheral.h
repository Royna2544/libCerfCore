#pragma once

#include "au1300_ohci_peripheral.h"

namespace cerfcore {

class Au1300Ohci1Peripheral final : public Au1300OhciPeripheral<1u> {
public:
    using Au1300OhciPeripheral::Au1300OhciPeripheral;
};

}
