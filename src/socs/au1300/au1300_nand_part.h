#pragma once

#include <cerfcore/peripherals/peripheral_base.h>
#include "../../peripherals/nand/small_page_nand_flash.h"

namespace cerfcore {

class Au1300NandPart : public Peripheral {
public:
    using Peripheral::Peripheral;
    virtual SmallPageNandFlash& Chip() = 0;
};

}
