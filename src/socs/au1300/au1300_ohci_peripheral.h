#pragma once

#include "au1300_id.h"
#include "au1300_model_host.h"
#include "au1300_usb_control.h"
#include "../../peripherals/usb/ohci_host_controller.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>

namespace cerfcore {

template<uint32_t kIndex>
class Au1300OhciPeripheral : public OhciHostController {
public:
    using OhciHostController::OhciHostController;
    bool ShouldRegister() override { return emu_.Get<BoardContext>().GetSocId() == SocId::Au1300; }
    uint32_t MmioBase() const override { return 0x14020400u + kIndex * 0x400u; }
    void OnReady() override {
        OhciHostController::OnReady();
        emu_.Get<Au1300UsbControl>().RegisterClockListener(kIndex, [this](bool on) { SetClocked(on); });
    }

protected:
    uint32_t ReadBus(uint32_t pa, uint32_t width) override {
        return emu_.Get<Au1300ModelHost>().ReadPhysical(pa, width);
    }
    void WriteBus(uint32_t pa, uint32_t value, uint32_t width) override {
        emu_.Get<Au1300ModelHost>().WritePhysical(pa, value, width);
    }
    void SetInterrupt(bool level) override {
        emu_.Get<Au1300UsbControl>().SetOhciInterrupt(kIndex, level);
    }
};

}
