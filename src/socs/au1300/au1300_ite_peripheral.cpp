#include "au1300_id.h"
#include "../../peripherals/peripheral_dispatcher.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/peripherals/peripheral_base.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>

#include <array>

namespace cerfcore {
namespace {

class Au1300ItePeripheral : public Peripheral {
public:
    using Peripheral::Peripheral;
    bool ShouldRegister() override {
        return emu_.Get<BoardContext>().GetSocId() == SocId::Au1300;
    }
    void OnReady() override {
        emu_.Get<PeripheralDispatcher>().Register(this);
        emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) { startup_writes_ = {}; });
    }
    uint32_t MmioBase() const override { return 0x14010000u; }
    uint32_t MmioSize() const override { return 0x714u; }
    void WriteWord(uint32_t address, uint32_t value) override {
        const auto offset = address - MmioBase();
        if (offset == 0x700u && value == 1u) startup_writes_[0u] = value;
        else if (offset == 0x704u && value == 2u) startup_writes_[1u] = value;
        else HaltUnsupportedAccess("WriteWord", address, value);
    }
    void SaveState(StateWriter& w) override { w.Write("startup_writes", startup_writes_); }
    void RestoreState(StateReader& r) override {
        r.Read("startup_writes", startup_writes_);
        if (startup_writes_[0u] > 1u || (startup_writes_[1u] != 0u && startup_writes_[1u] != 2u)) {
            r.Reject("Au1300 ITE startup state");
        }
    }

private:
    std::array<uint32_t, 2u> startup_writes_{};
};

}
REGISTER_SERVICE(Au1300ItePeripheral);
}
