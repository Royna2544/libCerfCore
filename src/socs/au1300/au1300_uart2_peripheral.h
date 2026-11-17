#pragma once

#include "au1300_uart_peripheral.h"

namespace cerfcore {

class Au1300Uart2Peripheral final : public Au1300UartPeripheral<0x10102000u, 2u> {
public:
    using Au1300UartPeripheral::Au1300UartPeripheral;
};

}
