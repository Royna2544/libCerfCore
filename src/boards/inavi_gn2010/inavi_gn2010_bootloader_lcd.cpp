#include "inavi_gn2010_family_id.h"
#include "../../socs/au1300/au1300_lcd.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/log.h>
#include <cerfcore/socs/guest_cpu_reset.h>

namespace cerfcore {
namespace {

class InaviGn2010BootloaderLcd : public Service {
public:
    using Service::Service;
    bool ShouldRegister() override {
        return emu_.Get<BoardContext>().GetBoardFamilyId() == BoardFamilyId::InaviGn2010;
    }
    void OnReady() override {
        Preset();
        emu_.Get<GuestCpuReset>().RegisterResetReleaseListener([this] { Preset(); });
    }

private:
    void Preset() {
        auto& lcd = emu_.Get<Au1300Lcd>();
        lcd.WriteWord(0x1500000Cu, 0x001C1E04u);
        lcd.WriteWord(0x15000010u, 0x00100E03u);
        lcd.WriteWord(0x15000014u, 0x00018001u);
        lcd.WriteWord(0x15000050u, 0x00FFFFFFu);
        lcd.WriteWord(0x15000054u, 0x2F2F2F2Fu);
        lcd.WriteWord(0x15000004u, 0x98F9DF80u);
        LOG(Board, "iNavi LCD handoff screen=98F9DF80 h=001C1E04 v=00100E03 clock=00018001 provisional=1\n");
    }
};

}
REGISTER_SERVICE(InaviGn2010BootloaderLcd);
}
