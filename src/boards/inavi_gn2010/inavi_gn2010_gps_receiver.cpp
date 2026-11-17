#include "inavi_gn2010_family_id.h"
#include "../../peripherals/serial/serial_endpoint.h"
#include "../../socs/au1300/au1300_uart0_peripheral.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/log.h>
#include <cerfcore/host/host_nmea_feed.h>
#include <cerfcore/state/state_stream.h>

#include <charconv>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <string_view>

namespace cerfcore {
namespace {

class InaviGn2010GpsReceiver : public Service, public SerialEndpoint {
public:
    using Service::Service;

    bool ShouldRegister() override {
        return emu_.Get<BoardContext>().GetBoardFamilyId() == BoardFamilyId::InaviGn2010;
    }

    void OnReady() override {
        auto& uart = emu_.Get<Au1300Uart0Peripheral>();
        BindUart(uart);
        uart.SetEndpoint(this);
        emu_.Get<HostNmeaFeed>().SetReceiver([this](const std::string& line) { Forward(line); });
        LOG(Periph, "iNavi GPS: u-blox receiver on UART0, fed by HostNmeaFeed\n");
    }

    void OnShutdown() override { emu_.Get<HostNmeaFeed>().SetReceiver(nullptr); }

    void OnGuestTx(const uint8_t* data, size_t n) override {
        for (size_t i = 0; i < n; ++i) {
            const char c = static_cast<char>(data[i]);
            if (c == '$') command_.clear();
            if (c == '\n') {
                ApplyCommand(command_);
                command_.clear();
                continue;
            }
            if (command_.size() < kCommandMax && (!command_.empty() || c == '$')) command_ += c;
        }
    }

    void SaveState(StateWriter& w) override {
        std::lock_guard<std::mutex> lk(mu_);
        w.Write("rate_count", static_cast<uint32_t>(rates_.size()));
        for (const auto& [id, rate] : rates_) {
            w.Write("rate_id", id);
            w.Write("rate_value", rate);
        }
    }

    void RestoreState(StateReader& r) override {
        std::lock_guard<std::mutex> lk(mu_);
        uint32_t count = 0u;
        r.Read("rate_count", count);
        if (count > kRateMax) r.Reject("iNavi GPS rate table size=%u", count);
        rates_.clear();
        sent_.clear();
        for (uint32_t i = 0; i < count; ++i) {
            uint32_t id   = 0u;
            uint32_t rate = 0u;
            r.Read("rate_id", id);
            r.Read("rate_value", rate);
            rates_[id] = rate;
        }
    }

private:
    static constexpr size_t   kCommandMax = 96u;
    static constexpr uint32_t kRateMax    = 64u;

    static uint32_t Formatter(std::string_view id) {
        if (id.size() != 3u) return 0u;
        return static_cast<uint32_t>(static_cast<uint8_t>(id[0])) |
               (static_cast<uint32_t>(static_cast<uint8_t>(id[1])) << 8u) |
               (static_cast<uint32_t>(static_cast<uint8_t>(id[2])) << 16u);
    }

    void ApplyCommand(std::string_view line) {
        while (!line.empty() && line.back() == '\r') line.remove_suffix(1u);
        const size_t star = line.find('*');
        if (line.substr(0u, 9u) != "$PUBX,40," || star == std::string_view::npos) return;
        std::string_view fields = line.substr(9u, star - 9u);
        std::string_view id     = fields.substr(0u, fields.find(','));
        uint32_t         rate   = 0u;
        size_t           at     = id.size();
        for (uint32_t field = 0u; field < 2u && at != std::string_view::npos; ++field) {
            const size_t next = fields.find(',', at + 1u);
            if (field == 1u) {
                const std::string_view value = fields.substr(at + 1u, next - at - 1u);
                if (std::from_chars(value.data(), value.data() + value.size(), rate).ec != std::errc{}) return;
            }
            at = next;
        }
        const uint32_t key = Formatter(id);
        if (key == 0u) return;
        std::lock_guard<std::mutex> lk(mu_);
        if (rates_.size() >= kRateMax && !rates_.contains(key)) return;
        rates_[key] = rate;
        LOG(Periph, "iNavi GPS: guest set %.*s output rate %u\n", static_cast<int>(id.size()),
            id.data(), rate);
    }

    void Forward(const std::string& line) {
        const uint32_t id = Formatter(std::string_view(line).substr(3u, 3u));
        {
            std::lock_guard<std::mutex> lk(mu_);
            const auto it = rates_.find(id);
            if (it != rates_.end()) {
                if (it->second == 0u) return;
                if (sent_[id]++ % it->second != 0u) return;
            }
        }
        uart_->PushRx(reinterpret_cast<const uint8_t*>(line.data()), line.size());
    }

    std::mutex                      mu_;
    std::map<uint32_t, uint32_t> rates_;
    std::map<uint32_t, uint32_t> sent_;
    std::string                     command_;
};

}
REGISTER_SERVICE(InaviGn2010GpsReceiver);
}
