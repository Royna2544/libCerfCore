#include "au1300_vss_peripheral.h"

#include "../../boards/inavi_gn2010/inavi_gn2010_family_id.h"
#include "../../peripherals/peripheral_dispatcher.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>

namespace cerfcore {

REGISTER_SERVICE(Au1300VssPeripheral);

bool Au1300VssPeripheral::ShouldRegister() {
    return emu_.Get<BoardContext>().GetBoardFamilyId() == BoardFamilyId::InaviGn2010;
}

void Au1300VssPeripheral::OnReady() {
    emu_.Get<PeripheralDispatcher>().Register(this);
    emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind kind) {
        if (kind == ResetLineKind::Rtc) registers_ = {};
    });
}

uint32_t Au1300VssPeripheral::ReadWord(uint32_t address) {
    const auto offset = address - MmioBase();
    if (offset >= MmioSize() || offset % 12u != 4u) HaltUnsupportedAccess("ReadWord", address, 0u);
    return registers_[offset / 12u][1u];
}

void Au1300VssPeripheral::WriteWord(uint32_t address, uint32_t value) {
    const auto offset = address - MmioBase();
    if (offset >= MmioSize() || (offset & 3u) != 0u) HaltUnsupportedAccess("WriteWord", address, value);
    constexpr std::array<uint32_t, 3u> masks = {0x1FFFFFFu, 3u, 0x1Fu};
    const auto index = (offset % 12u) / 4u;
    if ((value & ~masks[index]) != 0u) HaltUnsupportedAccess("WriteWord", address, value);
    registers_[offset / 12u][index] = value;
}

bool Au1300VssPeripheral::Enabled(Block block) const {
    const auto& r = registers_.at(static_cast<uint32_t>(block));
    return (r[0u] & 1u) != 0u && r[1u] == 2u && r[2u] == 0x1Fu;
}

void Au1300VssPeripheral::SaveState(StateWriter& w) {
    for (const auto& block : registers_) w.Write("block", block);
}

void Au1300VssPeripheral::RestoreState(StateReader& r) {
    for (auto& block : registers_) {
        r.Read("block", block);
        if ((block[0u] & ~0x1FFFFFFu) != 0u || (block[1u] & ~3u) != 0u || (block[2u] & ~0x1Fu) != 0u) {
            r.Reject("Au1300 VSS register state");
        }
    }
}

}
