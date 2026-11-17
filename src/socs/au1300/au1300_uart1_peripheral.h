#pragma once

#include "au1300_uart_peripheral.h"

namespace cerfcore {

class Au1300Uart1Peripheral final : public Au1300UartPeripheral<0x10101000u, 1u> {
public:
    using Au1300UartPeripheral::Au1300UartPeripheral;
};

}
