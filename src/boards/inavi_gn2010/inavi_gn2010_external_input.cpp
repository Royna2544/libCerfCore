#include "inavi_gn2010_family_id.h"
#include "../../peripherals/floating_bus.h"
#include "../../peripherals/peripheral_dispatcher.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/peripherals/peripheral_base.h>

namespace cerfcore {

namespace {

class InaviGn2010ExternalInput : public Peripheral {
public:
    using Peripheral::Peripheral;

    bool ShouldRegister() override {
        return emu_.Get<BoardContext>().GetBoardFamilyId() == BoardFamilyId::InaviGn2010;
    }
    void OnReady() override { emu_.Get<PeripheralDispatcher>().Register(this); }
    uint32_t MmioBase() const override { return 0x1FE00000u; }
    uint32_t MmioSize() const override { return 0x1000u; }
    uint8_t ReadByte(uint32_t address) override { return input_.Read8("iNavi external input", address); }
    uint16_t ReadHalf(uint32_t address) override { return input_.Read16("iNavi external input", address); }
    uint32_t ReadWord(uint32_t address) override { return input_.Read32("iNavi external input", address); }

private:
    FloatingBus input_;
};

}

REGISTER_SERVICE(InaviGn2010ExternalInput);

}
