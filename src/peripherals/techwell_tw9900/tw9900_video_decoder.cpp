#include "tw9900_video_decoder.h"

#include "../../boards/inavi_gn2010/inavi_gn2010_family_id.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/state/state_stream.h>

namespace cerfcore {

REGISTER_SERVICE(Tw9900VideoDecoder);

bool Tw9900VideoDecoder::ShouldRegister() {
    return emu_.Get<BoardContext>().GetBoardFamilyId() == BoardFamilyId::InaviGn2010;
}

void Tw9900VideoDecoder::OnReady() {
    regs_.fill(0u);
    regs_[0x00u] = 0x00u;
    regs_[0x01u] = 0x83u;
    regs_[0x02u] = 0x40u;
    regs_[0x03u] = 0x24u;
}

void Tw9900VideoDecoder::TxnStart(uint8_t) {
    have_pointer_ = false;
}

void Tw9900VideoDecoder::TxnWriteByte(uint8_t, uint8_t byte) {
    if (!have_pointer_) {
        pointer_      = byte;
        have_pointer_ = true;
        return;
    }
    regs_[pointer_] = byte;
    pointer_        = static_cast<uint8_t>(pointer_ + 1u);
}

uint8_t Tw9900VideoDecoder::TxnReadByte(uint8_t) {
    const uint8_t value = regs_[pointer_];
    pointer_            = static_cast<uint8_t>(pointer_ + 1u);
    return value;
}

void Tw9900VideoDecoder::SaveState(StateWriter& w) {
    w.Write("tw9900_regs", regs_);
    w.Write("tw9900_pointer", pointer_);
    w.Write("tw9900_have_pointer", have_pointer_);
}

void Tw9900VideoDecoder::RestoreState(StateReader& r) {
    r.Read("tw9900_regs", regs_);
    r.Read("tw9900_pointer", pointer_);
    r.Read("tw9900_have_pointer", have_pointer_);
}

}
