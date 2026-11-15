#include "inavi_gn2010_family_id.h"
#include "../../execution/mips_engine_bridge.h"

#include <cerfcore/boards/board_context.h>
#include <cerfcore/boot/rom_parser_service.h>
#include <cerfcore/core/byte_order.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/core/device_config.h>
#include <cerfengine/mips/mips_cpu.h>
#include <cerfengine/unmapped_windows.h>

namespace cerfcore {

namespace {

class InaviGn2010BootEntry : public Service {
public:
    using Service::Service;

    bool ShouldRegister() override {
        return emu_.Get<BoardContext>().GetBoardFamilyId() == BoardFamilyId::InaviGn2010;
    }

    void OnReady() override {
        if (emu_.Get<DeviceConfig>().guest_additions) {
            emu_.Get<Fatal>().Die("iNavi NAND-backed Guest Additions injection is unmodeled");
        }
        const auto& rom = emu_.Get<RomParserService>().Primary();
        for (const auto& xip : rom.xips) {
            const auto& header = xip.toc.romhdr;
            if (!header.ImageContains(rom.entry_va) || header.physfirst < xip.load_offset) continue;
            const size_t offset = header.physfirst - xip.load_offset;
            if (offset > rom.flat.size() || rom.flat.size() - offset < 20u) break;
            const auto* code = rom.flat.data() + offset;
            const uint32_t upper = cerf::le::U32(code, 4u);
            const uint32_t lower = cerf::le::U32(code, 8u);
            const uint32_t target = (upper << 16u) | (lower & 0xFFFFu);
            if ((upper & 0xFFFF0000u) != 0x3C1A0000u ||
                (lower & 0xFFFF0000u) != 0x375A0000u ||
                cerf::le::U32(code, 12u) != 0x03400008u || cerf::le::U32(code, 16u) != 0u ||
                !cerfengine::UnmappedWindows::Contains(target) ||
                !cerfengine::UnmappedWindows::Contains(rom.entry_va) ||
                cerfengine::UnmappedWindows::Pa(target) != cerfengine::UnmappedWindows::Pa(rom.entry_va)) {
                break;
            }
            entry_ = header.physfirst + 4u;
            auto& bridge = emu_.Get<MipsEngineBridge>();
            state_ = bridge.Engine().Get<cerfengine::MipsCpu>().State();
            bridge.RegisterPostResetListener([this] { ApplyEntry(); });
            ApplyEntry();
            return;
        }
        emu_.Get<Fatal>().Die("iNavi primary XIP has no supported launch trampoline");
    }

private:
    void ApplyEntry() { state_->pc = entry_; }
    cerfengine::MipsCpuState* state_ = nullptr;
    uint32_t entry_ = 0u;
};

}

REGISTER_SERVICE(InaviGn2010BootEntry);

}
