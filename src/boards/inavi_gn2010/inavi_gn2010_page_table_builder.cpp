#include "inavi_gn2010_family_id.h"
#include "../unmapped_window_page_table_builder.h"

#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/cpu/memory_kind.h>

namespace cerfcore {

namespace {

class InaviGn2010PageTableBuilder : public UnmappedWindowPageTableBuilder {
public:
    using UnmappedWindowPageTableBuilder::UnmappedWindowPageTableBuilder;

    bool ShouldRegister() override {
        return emu_.Get<BoardContext>().GetBoardFamilyId() == BoardFamilyId::InaviGn2010;
    }

    std::vector<DramRegion> CachedDramRegions() const override {
        return {{0x80000000u, 0u, 0x10000000u}};
    }

    std::vector<BackedRegion> BackedMemoryRegions() const override {
        return {{0x80000000u, 0u, 0x10000000u, MemoryKind::kRam}};
    }

    std::vector<DramRegion> MappedVaSpans() const override {
        return {{0x80000000u, 0u, 0x20000000u}};
    }
};

}

REGISTER_SERVICE_AS(InaviGn2010PageTableBuilder, PageTableBuilder);

}
