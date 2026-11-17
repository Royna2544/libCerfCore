#pragma once

#include "au1300_uart_peripheral.h"

namespace cerfcore {

class Au1300Uart3Peripheral final : public Au1300UartPeripheral<0x10103000u, 3u> {
public:
    using Au1300UartPeripheral::Au1300UartPeripheral;
};

}
