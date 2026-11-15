#include "inavi_gn2010_family_id.h"
#include "../../peripherals/wolfson_wm97xx/wm97xx_codec.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/host/touch_input.h>

#include <algorithm>

namespace cerfcore {
namespace {

class InaviGn2010TouchInput : public TouchInput {
public:
    using TouchInput::TouchInput;
    bool ShouldRegister() override {
        return emu_.Get<BoardContext>().GetBoardFamilyId() == BoardFamilyId::InaviGn2010;
    }
    void OnReady() override {
        codec_ = dynamic_cast<Wm97xxCodec*>(&emu_.Get<Ac97Codec>());
        if (!codec_) emu_.Get<Fatal>().Die("iNavi touch codec type");
    }
    void OnPenDown(int x, int y) override { Position(x, y); codec_->QueuePenEdge(true); }
    void OnPenMove(int x, int y) override { Position(x, y); }
    void OnPenUp(int x, int y) override { Position(x, y); codec_->QueuePenEdge(false); }
    void OnCaptureLost() override { codec_->QueuePenEdge(false); }

private:
    void Position(int x, int y) {
        const int64_t horizontal = std::clamp(x, 0, 799);
        const int64_t vertical = std::clamp(y, 0, 479);
        const int64_t raw1 = (8358 * horizontal - 4017448 * vertical + 1968844402 + 500000) / 1000000;
        const int64_t raw2 = (1237506 * horizontal - 3493 * vertical + 18635850 + 500000) / 1000000;
        codec_->SetPenPosition(static_cast<uint16_t>(std::clamp<int64_t>(raw2 * 4, 0, 4095)),
                               static_cast<uint16_t>(std::clamp<int64_t>(raw1 * 2, 0, 4095)));
    }
    Wm97xxCodec* codec_ = nullptr;
};

}
REGISTER_SERVICE_AS(InaviGn2010TouchInput, TouchInput);
}
