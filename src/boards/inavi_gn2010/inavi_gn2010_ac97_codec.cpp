#include "inavi_gn2010_family_id.h"
#include "../../peripherals/wolfson_wm9705/wm9705_codec.h"
#include "../../socs/au1300/au1300_model_host.h"
#include "../../socs/au1300/au1300_gpic_model.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/log.h>

namespace cerfcore {
namespace {

class InaviGn2010Ac97Codec : public Wm9705Codec {
public:
    using Wm9705Codec::Wm9705Codec;
    bool ShouldRegister() override {
        return emu_.Get<BoardContext>().GetBoardFamilyId() == BoardFamilyId::InaviGn2010;
    }
    void OnReady() override {
        Wm9705Codec::OnReady();
        SetPenLineObserver([this](bool down) { emu_.Get<Au1300ModelHost>().Gpic().SetLevel(21u, down); });
        LOG(Boot, "iNavi AC97: provisional WM9705-compatible codec; physical codec identity unverified\n");
        LOG(Boot, "iNavi BMON: provisional ADC=1738, 4.2V battery with 3.3V reference\n");
    }

protected:
    uint16_t ConversionData(uint8_t tag) override {
        if (tag == 6u) return 1738u;
        return Wm9705Codec::ConversionData(tag);
    }
};

}
REGISTER_SERVICE_AS(InaviGn2010Ac97Codec, Ac97Codec);
}
