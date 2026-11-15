#include "inavi_gn2010_family_id.h"
#include "../../socs/au1300/au1300_nand_control.h"
#include "../../socs/au1300/au1300_nand_part.h"
#include "../../peripherals/nand/volatile_small_page_nand_store.h"
#include "../../peripherals/nand/samsung_nand_unique_id.h"
#include "../../peripherals/peripheral_dispatcher.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/cerf_paths.h>
#include <cerfcore/core/device_config.h>
#include <cerfcore/socs/guest_cpu_reset.h>

namespace cerfcore {

namespace {

constexpr uint8_t kId[] = {0xECu, 0x76u, 0x5Au, 0x3Fu};
constexpr SmallPageNandGeometry kGeometry = {4096u, 3u, kId, sizeof kId, true};

class InaviGn2010NandFlash final : public Au1300NandPart {
public:
    using Au1300NandPart::Au1300NandPart;

    bool ShouldRegister() override {
        return emu_.Get<BoardContext>().GetBoardFamilyId() == BoardFamilyId::InaviGn2010;
    }
    void OnReady() override {
        const auto& config = emu_.Get<DeviceConfig>();
        const auto path = ResolveDeviceFile(config.device_name, config.storage_nand);
        store_.Open(path);
        uid_.Open(path + ".uid", path + ".otp");
        chip_.SetWriteProtect(false);
        emu_.Get<PeripheralDispatcher>().Register(this);
        emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind kind) {
            if (kind == ResetLineKind::Rtc) {
                chip_.PowerUp();
                chip_.SetWriteProtect(false);
                uid_.Reset();
            }
        });
    }
    uint32_t MmioBase() const override { return 0x1C000000u; }
    uint32_t MmioSize() const override { return 0x24u; }
    SmallPageNandFlash& Chip() override { return chip_; }

    uint8_t ReadByte(uint32_t address) override {
        if (address != MmioBase() + 0x20u) HaltUnsupportedAccess("ReadByte", address, 0u);
        const uint8_t value = ReadData();
        emu_.Get<Au1300NandControl>().EndAccess();
        return value;
    }
    void WriteByte(uint32_t address, uint8_t value) override {
        switch (address - MmioBase()) {
        case 0u:
            if (!uid_.Command(value)) chip_.Command(value);
            break;
        case 4u:
            if (uid_.Active()) uid_.Address(value);
            else chip_.Address(value);
            break;
        case 0x20u:
            if (uid_.Active()) HaltUnsupportedAccess("UID WriteData", address, value);
            chip_.WriteData(value);
            break;
        default: HaltUnsupportedAccess("WriteByte", address, value);
        }
        emu_.Get<Au1300NandControl>().EndAccess();
    }
    uint32_t ReadWord(uint32_t address) override {
        if (address != MmioBase() + 0x20u) HaltUnsupportedAccess("ReadWord", address, 0u);
        uint32_t value = 0u;
        for (uint32_t i = 0u; i < 4u; ++i) value |= static_cast<uint32_t>(ReadData()) << (8u * i);
        emu_.Get<Au1300NandControl>().EndAccess();
        return value;
    }
    void WriteWord(uint32_t address, uint32_t value) override {
        if (address != MmioBase() + 0x20u) HaltUnsupportedAccess("WriteWord", address, value);
        if (uid_.Active()) HaltUnsupportedAccess("UID WriteWord", address, value);
        for (uint32_t i = 0u; i < 4u; ++i) chip_.WriteData(static_cast<uint8_t>(value >> (8u * i)));
        emu_.Get<Au1300NandControl>().EndAccess();
    }
    void SaveState(StateWriter& w) override {
        chip_.SaveState(w);
        store_.SaveState(w);
        uid_.SaveState(w);
    }
    void RestoreState(StateReader& r) override {
        chip_.RestoreState(r);
        store_.RestoreState(r);
        uid_.RestoreState(r);
    }

private:
    uint8_t ReadData() { return uid_.Active() ? uid_.ReadData() : chip_.ReadData(); }
    VolatileSmallPageNandStore store_{emu_, kGeometry.blocks};
    SmallPageNandFlash chip_{emu_, "Samsung K9F1208U0C", kGeometry, store_};
    SamsungNandUniqueId uid_{emu_};
};

}

REGISTER_SERVICE_AS(InaviGn2010NandFlash, Au1300NandPart);

}
