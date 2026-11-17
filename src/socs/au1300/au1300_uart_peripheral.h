#pragma once

#include "au1300_peripheral.h"
#include "../uart_rx_line.h"
#include "../../execution/engine_bridge.h"
#include "../../peripherals/serial/serial_device_clock_timer.h"
#include "../../peripherals/serial/serial_endpoint_port.h"
#include "../../peripherals/serial/serial_line.h"
#include <cerfengine/host_request_channel.h>

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

namespace cerfcore {

template<uint32_t kBase, uint32_t kIndex>
class Au1300UartPeripheral : public Au1300Peripheral<Au1300UartModel, kBase, 0x1000u, kIndex>,
                             public SerialLine {
    using Base = Au1300Peripheral<Au1300UartModel, kBase, 0x1000u, kIndex>;

public:
    explicit Au1300UartPeripheral(CerfEmulator& emu)
        : Base(emu),
          endpoint_timer_(emu, [this] { port_.OnTimer(); }),
          port_(*this, endpoint_timer_,
                [this] { return SerialControlLines{dtr_.load(), rts_.load()}; }),
          wire_(kNames[kIndex]) {}

    void OnReady() override {
        Base::OnReady();
        endpoint_timer_.AttachLineTimer();
        Au1300UartModel& model = this->Registers();
        model.SetHooks(Au1300UartModel::Hooks{
            [this](uint8_t value) { return port_.DeliverTx(&value, 1u, true); },
            [this] { OnWireIdle(); },
            [this] { RefreshLineConfig(true); },
            [this] { RefreshControlLines(true); }});
        requests_ = &this->emu_.template Get<EngineBridge>().Engine()
                         .template Get<cerfengine::HostRequestChannel>();
        requests_->RegisterListener([this] { OnHostRequest(); });
        RefreshLineConfig(false);
        RefreshControlLines(false);
    }

    void OnShutdown() override {
        Base::OnShutdown();
        port_.OnShutdown();
    }

    void PushRx(const uint8_t* data, size_t n) override {
        if (n == 0u || !wire_.Push(data, n)) return;
        rx_busy_.store(true);
        port_.MarkRx();
        requests_->Request();
    }
    bool RxEmpty() const override { return !rx_busy_.load(); }
    void SetRxDrainCallback(RxDrainFn cb) override { port_.SetRxDrainCallback(std::move(cb)); }

    void SetModemInputs(bool cts, bool dsr, bool ri, bool dcd) override {
        modem_inputs_.store((cts ? 0x10u : 0u) | (dsr ? 0x20u : 0u) | (ri ? 0x40u : 0u) |
                            (dcd ? 0x80u : 0u));
        modem_dirty_.store(true);
        requests_->Request();
    }

    LineConfig GetLineConfig() const override {
        std::lock_guard<std::mutex> lk(line_mu_);
        return line_cfg_;
    }
    void SetLineConfigCallback(LineConfigFn cb) override {
        port_.SetLineConfigCallback(std::move(cb));
    }
    void SetEndpoint(SerialEndpoint* ep) override { port_.SetEndpoint(ep); }
    void ArmEndpointTimer(int64_t delay_ns) override { port_.ArmTimer(delay_ns); }
    void DisarmEndpointTimer() override { port_.DisarmTimer(); }

    void SaveState(StateWriter& w) override {
        port_.SaveCradleState(w);
        Base::SaveState(w);
        endpoint_timer_.SaveLineTimer(w, "endpoint_timer_");
        port_.SaveEndpointState(w);
    }
    void RestoreState(StateReader& r) override {
        port_.RestoreCradleState(r);
        Base::RestoreState(r);
        endpoint_timer_.RestoreLineTimer(r, "endpoint_timer_");
        port_.RestoreEndpointState(r);
    }
    void PostRestore() override {
        Base::PostRestore();
        endpoint_timer_.ResumeLineTimer();
        rx_busy_.store(!this->Registers().WireIdle() || !wire_.WireEmpty());
        RefreshLineConfig(false);
        RefreshControlLines(false);
        port_.PostRestore();
    }

private:
    static constexpr std::array<const char*, 4u> kNames{"UART0", "UART1", "UART2", "UART3"};

    void OnHostRequest() {
        Au1300UartModel& model = this->Registers();
        if (modem_dirty_.exchange(false)) model.SetModemInputs(modem_inputs_.load());
        const std::vector<uint8_t> bytes = wire_.TakeWire();
        if (!bytes.empty()) model.AcceptWire(bytes);
    }

    void OnWireIdle() {
        if (!wire_.WireEmpty()) {
            requests_->Request();
            return;
        }
        rx_busy_.store(false);
        port_.NotifyRxDrained();
    }

    void RefreshLineConfig(bool notify) {
        const Au1300UartModel& model = this->Registers();
        const uint32_t         lcr   = model.LineControl();
        LineConfig             cfg;
        cfg.baud      = static_cast<uint32_t>(this->emu_.template Get<Au1300ModelHost>().Sys().CpuHz() /
                                              model.BitCycles());
        cfg.data_bits = static_cast<uint8_t>(5u + (lcr & 3u));
        cfg.stop      = (lcr & 4u) == 0u ? LineConfig::Stop::One
                      : cfg.data_bits == 5u ? LineConfig::Stop::OnePointFive
                                            : LineConfig::Stop::Two;
        if ((lcr & 8u) == 0u) cfg.parity = LineConfig::Parity::None;
        else if ((lcr & 0x20u) != 0u) cfg.parity = (lcr & 0x10u) != 0u ? LineConfig::Parity::Space
                                                                         : LineConfig::Parity::Mark;
        else cfg.parity = (lcr & 0x10u) != 0u ? LineConfig::Parity::Even : LineConfig::Parity::Odd;
        {
            std::lock_guard<std::mutex> lk(line_mu_);
            line_cfg_ = cfg;
        }
        if (notify) port_.NotifyLineConfig(cfg);
    }

    void RefreshControlLines(bool notify) {
        const Au1300UartModel& model = this->Registers();
        dtr_.store(model.Dtr());
        rts_.store(model.Rts());
        if (notify) port_.NotifyControlLines();
    }

    SerialDeviceClockTimer           endpoint_timer_;
    SerialEndpointPort               port_;
    UartRxLine                       wire_;
    cerfengine::HostRequestChannel*  requests_ = nullptr;
    std::atomic<bool>                rx_busy_{false};
    std::atomic<uint32_t>            modem_inputs_{0u};
    std::atomic<bool>                modem_dirty_{false};
    std::atomic<bool>                dtr_{false};
    std::atomic<bool>                rts_{false};
    mutable std::mutex               line_mu_;
    LineConfig                       line_cfg_;
};

}
