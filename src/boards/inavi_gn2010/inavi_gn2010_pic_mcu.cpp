#include "inavi_gn2010_pic_mcu.h"

#include "inavi_gn2010_family_id.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/core/log.h>

namespace cerfcore {

REGISTER_SERVICE(InaviGn2010PicMcu);

bool InaviGn2010PicMcu::ShouldRegister() {
    return emu_.Get<BoardContext>().GetBoardFamilyId() == BoardFamilyId::InaviGn2010;
}

void InaviGn2010PicMcu::TxnStart(uint8_t) {}

void InaviGn2010PicMcu::TxnWriteByte(uint8_t, uint8_t byte) {
    LOG(Periph, "iNavi PIC: command byte 0x%02X\n", byte);
}

uint8_t InaviGn2010PicMcu::TxnReadByte(uint8_t) {
    emu_.Get<Fatal>().Die("iNavi PIC: status read without a modeled PIC interrupt event");
}

}
