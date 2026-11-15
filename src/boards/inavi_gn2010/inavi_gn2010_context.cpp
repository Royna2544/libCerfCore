#include "inavi_gn2010_family_id.h"

#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>

namespace cerfcore {

namespace {

class InaviGn2010Context : public BoardContext {
public:
    using BoardContext::BoardContext;

    std::string_view GetBoardFamilyId() const override { return BoardFamilyId::InaviGn2010; }
};

}

REGISTER_SERVICE_AS(InaviGn2010Context, BoardContext);

}
