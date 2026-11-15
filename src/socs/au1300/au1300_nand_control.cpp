#include "au1300_nand_control.h"

#include "au1300_id.h"
#include "au1300_model_host.h"
#include "au1300_gpic_model.h"
#include "au1300_nand_part.h"
#include "../../peripherals/peripheral_dispatcher.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/state/state_stream.h>
#include <cerfcore/socs/guest_cpu_reset.h>

namespace cerfcore {

REGISTER_SERVICE(Au1300NandControl);

bool Au1300NandControl::ShouldRegister() {
    return emu_.Get<BoardContext>().GetSocId() == SocId::Au1300;
}

void Au1300NandControl::OnReady() {
    emu_.Get<PeripheralDispatcher>().Register(this);
    auto& reset = emu_.Get<GuestCpuReset>();
    reset.RegisterResetListener([this](ResetLineKind) {
        control_ = 0u;
        emu_.Get<Au1300NandPart>().Chip().Deselect();
    });
    reset.RegisterResetReleaseListener([this] { PostRestore(); });
}

uint32_t Au1300NandControl::MmioBase() const { return 0x14001100u; }
uint32_t Au1300NandControl::MmioSize() const { return 8u; }

uint32_t Au1300NandControl::ReadWord(uint32_t address) {
    if (address == MmioBase()) return control_;
    if (address == MmioBase() + 4u) return 1u;
    HaltUnsupportedAccess("ReadWord", address, 0u);
}

void Au1300NandControl::WriteWord(uint32_t address, uint32_t value) {
    if (address != MmioBase()) HaltUnsupportedAccess("WriteWord", address, value);
    if ((value & ~0x110u) != 0u) {
        emu_.Get<Fatal>().Die("Au1300 NAND control=%08X", value);
    }
    const bool deselect = (control_ & 0x10u) != 0u && (value & 0x10u) == 0u;
    control_ = value;
    if (deselect) emu_.Get<Au1300NandPart>().Chip().Deselect();
    PostRestore();
}

void Au1300NandControl::EndAccess() {
    if ((control_ & 0x10u) == 0u) emu_.Get<Au1300NandPart>().Chip().Deselect();
}

void Au1300NandControl::SaveState(StateWriter& w) { w.Write("control", control_); }

void Au1300NandControl::RestoreState(StateReader& r) {
    r.Read("control", control_);
    if ((control_ & ~0x110u) != 0u) r.Reject("Au1300 NAND control=%08X", control_);
}

void Au1300NandControl::PostRestore() {
    emu_.Get<Au1300ModelHost>().Gpic().SetLevel(62u, (control_ & 0x100u) != 0u);
}

}
