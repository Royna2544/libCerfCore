#include "au1300_id.h"
#include "../../cpu/mips/mips_core_clock.h"

#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>

namespace cerfcore {

namespace {

class Au1300CoreClock : public MipsCoreClock {
public:
    using MipsCoreClock::MipsCoreClock;

    bool ShouldRegister() override { return emu_.Get<BoardContext>().GetSocId() == SocId::Au1300; }
    GuestCycleClock::Rate ResetRate() const override { return {792000000u, 1u}; }
    uint64_t CyclesPerCountTick() const override { return 1u; }
};

}

REGISTER_SERVICE_AS(Au1300CoreClock, MipsCoreClock);

}
