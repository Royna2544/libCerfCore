#pragma once

#include "au1300_uart_peripheral.h"

namespace cerfcore {

class Au1300Uart0Peripheral final : public Au1300UartPeripheral<0x10100000u, 0u> {
public:
    using Au1300UartPeripheral::Au1300UartPeripheral;
};

}
