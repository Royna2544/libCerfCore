#pragma once

#include "au1300_id.h"
#include "au1300_clock_model.h"
#include "au1300_dma_model.h"
#include "au1300_gpic_model.h"
#include "au1300_model_host.h"
#include "au1300_uart_model.h"
#include "../../peripherals/peripheral_dispatcher.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/peripherals/peripheral_base.h>
#include <cerfcore/socs/guest_cpu_reset.h>

#include <memory>
#include <type_traits>

namespace cerfcore {

template<class Model, uint32_t kBase, uint32_t kSize, uint32_t kIndex = 0u>
class Au1300Peripheral : public Peripheral {
public:
    using Peripheral::Peripheral;
    bool ShouldRegister() override {
        return emu_.Get<BoardContext>().GetSocId() == SocId::Au1300;
    }
    uint32_t MmioBase() const override { return kBase; }
    uint32_t MmioSize() const override { return kSize; }

    void OnReady() override {
        auto& host = emu_.Get<Au1300ModelHost>();
        if constexpr (std::is_same_v<Model, Au1300UartModel>) {
            model_ = std::make_unique<Model>(host, kIndex);
        } else {
            model_ = std::make_unique<Model>(host);
        }
        emu_.Get<PeripheralDispatcher>().Register(this);
        auto& reset = emu_.Get<GuestCpuReset>();
        reset.RegisterResetListener([this](ResetLineKind kind) {
            if constexpr (std::is_same_v<Model, Au1300ClockModel>) {
                model_->Reset(kind == ResetLineKind::Rtc);
            } else {
                model_->Reset();
            }
        });
        reset.RegisterResetReleaseListener([this] { model_->PostRestore(); });
    }
    void OnShutdown() override {
        if constexpr (std::is_same_v<Model, Au1300UartModel>) {
            if (model_) model_->FlushLine();
        }
    }
    uint32_t ReadWord(uint32_t address) override { return model_->Read(address - kBase); }
    void WriteWord(uint32_t address, uint32_t value) override {
        if constexpr (std::is_same_v<Model, Au1300GpicModel>) {
            emu_.Get<Au1300ModelHost>().Sys().Tick();
        }
        model_->Write(address - kBase, value);
    }
    uint8_t ReadByte(uint32_t address) override {
        if constexpr (std::is_same_v<Model, Au1300UartModel>) {
            if (address == kBase) return static_cast<uint8_t>(model_->Read(0u));
        }
        HaltUnsupportedAccess("ReadByte", address, 0u);
    }
    void WriteByte(uint32_t address, uint8_t value) override {
        if constexpr (std::is_same_v<Model, Au1300UartModel>) {
            if (address == kBase + 4u) {
                model_->Write(4u, value);
                return;
            }
        }
        HaltUnsupportedAccess("WriteByte", address, value);
    }
    void SaveState(StateWriter& w) override { model_->SaveState(w); }
    void RestoreState(StateReader& r) override { model_->RestoreState(r); }
    void PostRestore() override { model_->PostRestore(); }
    Model& Registers() { return *model_; }

private:
    std::unique_ptr<Model> model_;
};

}
