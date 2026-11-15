#include "inavi_gn2010_family_id.h"
#include "../../peripherals/mmc/sd_memory_card.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/cerf_paths.h>
#include <cerfcore/core/device_config.h>
#include <cerfcore/core/fatal.h>

namespace cerfcore {
namespace {

class InaviGn2010SdCard : public SdMemoryCard {
public:
    using SdMemoryCard::SdMemoryCard;
    bool ShouldRegister() override {
        if (emu_.Get<BoardContext>().GetBoardFamilyId() != BoardFamilyId::InaviGn2010) return false;
        const auto& media = emu_.Get<DeviceConfig>().bundled_sd_cards;
        for (const auto& entry : media) if (entry.insert_on_launch) return true;
        return false;
    }
    uint32_t SlotIndex() const override { return 1u; }

protected:
    std::string ImagePath() const override {
        const auto& config = emu_.Get<DeviceConfig>();
        return ResolveDeviceFile(config.device_name, Entry().file);
    }
    SdCardCid CardCid() const override {
        if (Entry().cid) return *Entry().cid;
        return {0xFFu, 'C', 'E', 'I', 'N', 'A', 'V', 'I', 0x10u, 0u, 0u, 0x25u, 0x44u, 0x01u, 0xABu, 0u};
    }

private:
    const BundledUsbMedia& Entry() const {
        const BundledUsbMedia* selected = nullptr;
        for (const auto& entry : emu_.Get<DeviceConfig>().bundled_sd_cards) {
            if (!entry.insert_on_launch) continue;
            if (selected) emu_.Get<Fatal>().UserError("iNavi SD slot has multiple launch images");
            selected = &entry;
        }
        if (!selected) emu_.Get<Fatal>().UserError("iNavi SD slot has no launch image");
        return *selected;
    }
};

}
REGISTER_SERVICE_AS(InaviGn2010SdCard, MmcCard);
}
