#include "mips_cp0_model.h"

namespace cerfcore {

cerfengine::MipsCp0Decision MipsCp0Model::Decide(uint32_t rd, uint32_t sel) {
    return sel == 0u ? Decide(rd) : cerfengine::MipsCp0Decision{};
}

}
