#include "inavi_gn2010_family_id.h"
#include "inavi_gn2010_pic_mcu.h"
#include "../../peripherals/bosch_smb380/smb380_accelerometer.h"
#include "../../peripherals/i2c/i2c_bit_bang_target.h"
#include "../../peripherals/rohm_bh1750/bh1750_light_sensor.h"
#include "../../peripherals/techwell_tw9900/tw9900_video_decoder.h"
#include "../../peripherals/yamaha_yas526b/yas526b_magnetometer.h"
#include "../../socs/au1300/au1300_gpic_model.h"
#include "../../socs/au1300/au1300_model_host.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/log.h>
#include <cerfcore/state/state_stream.h>

#include <array>
#include <optional>
#include <vector>

namespace cerfcore {
namespace {

class InaviGn2010I2cBus : public Service {
public:
    using Service::Service;

    bool ShouldRegister() override {
        return emu_.Get<BoardContext>().GetBoardFamilyId() == BoardFamilyId::InaviGn2010;
    }

    void OnReady() override {
        gpic_ = &emu_.Get<Au1300ModelHost>().Gpic();
        buses_[0].devices = {&emu_.Get<Smb380Accelerometer>(), &emu_.Get<Yas526bMagnetometer>()};
        buses_[1].devices = {&emu_.Get<Bh1750LightSensor>()};
        buses_[2].devices = {&emu_.Get<Tw9900VideoDecoder>(), &emu_.Get<InaviGn2010PicMcu>()};
        for (size_t i = 0u; i < buses_.size(); ++i) {
            auto& bus = buses_[i];
            bus.target.emplace([this, i](bool released) { DriveSda(buses_[i], released); },
                               bus.devices, [i](uint8_t addr, bool read) {
                                   LOG(Caution, "iNavi I2C bus %zu: no device answers 0x%02X (%s)\n",
                                       i, addr, read ? "read" : "write");
                               });
            gpic_->SetLevel(bus.scl_gpio, true);
            gpic_->SetLevel(bus.sda_gpio, true);
        }
        gpic_->AddOutputListener([this] {
            for (auto& bus : buses_) OnPins(bus);
        });
        gpic_->AddPinDeviceState([this](StateWriter& w) { SaveState(w); },
                                 [this](StateReader& r) { RestoreState(r); });
    }

private:
    struct Bus {
        Bus(uint32_t sda, uint32_t scl) : sda_gpio(sda), scl_gpio(scl) {}

        uint32_t                        sda_gpio;
        uint32_t                        scl_gpio;
        std::vector<I2cSlave*>          devices;
        std::optional<I2cBitBangTarget> target;
        bool                            sda_released = true;
    };

    void OnPins(Bus& bus) {
        const bool scl = !gpic_->DrivenLow(bus.scl_gpio);
        const bool sda = !gpic_->DrivenLow(bus.sda_gpio) && bus.sda_released;
        bus.target->OnLines(scl, sda);
    }

    void DriveSda(Bus& bus, bool released) {
        bus.sda_released = released;
        gpic_->SetLevel(bus.sda_gpio, released);
    }

    void SaveState(StateWriter& w) {
        for (auto& bus : buses_) {
            bus.target->SaveState(w);
            w.Write("i2c_sda_released", bus.sda_released);
            for (I2cSlave* device : bus.devices) device->SaveState(w);
        }
    }

    void RestoreState(StateReader& r) {
        for (auto& bus : buses_) {
            bus.target->RestoreState(r);
            r.Read("i2c_sda_released", bus.sda_released);
            for (I2cSlave* device : bus.devices) device->RestoreState(r);
        }
    }

    Au1300GpicModel*   gpic_ = nullptr;
    std::array<Bus, 3u> buses_{{{70u, 63u}, {56u, 73u}, {60u, 74u}}};
};

}
REGISTER_SERVICE(InaviGn2010I2cBus);
}
