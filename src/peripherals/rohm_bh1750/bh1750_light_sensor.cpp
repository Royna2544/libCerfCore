#include "bh1750_light_sensor.h"

#include "../../boards/inavi_gn2010/inavi_gn2010_family_id.h"
#include "../../execution/guest_cycle_clock.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/state/state_stream.h>

#include <cmath>

namespace cerfcore {

REGISTER_SERVICE(Bh1750LightSensor);

namespace {

constexpr uint8_t kPowerDown = 0x00u;
constexpr uint8_t kPowerOn = 0x01u;
constexpr uint8_t kContinuousHighResolution = 0x10u;
constexpr double kDefaultAmbientLux = 100.0;
constexpr double kCountsPerLux = 1.2;
constexpr int64_t kHighResolutionNs = 120000000;

}

bool Bh1750LightSensor::ShouldRegister() {
    return emu_.Get<BoardContext>().GetBoardFamilyId() == BoardFamilyId::InaviGn2010;
}

void Bh1750LightSensor::OnReady() {
    clock_ = &emu_.Get<GuestCycleClock>();
}

void Bh1750LightSensor::TxnStart(uint8_t) {
    read_index_ = 0u;
}

void Bh1750LightSensor::TxnWriteByte(uint8_t, uint8_t byte) {
    switch (byte) {
        case kPowerDown:
            powered_ = false;
            mode_ = kPowerDown;
            return;
        case kPowerOn:
            powered_ = true;
            mode_ = kPowerOn;
            return;
        case kContinuousHighResolution:
            powered_ = true;
            mode_ = byte;
            ready_ns_ = clock_->NowNs() + kHighResolutionNs;
            return;
        default:
            emu_.Get<Fatal>().Die("BH1750: unsupported command 0x%02X", byte);
    }
}

uint8_t Bh1750LightSensor::TxnReadByte(uint8_t) {
    if (read_index_ >= 2u) emu_.Get<Fatal>().Die("BH1750: read past measurement result");
    if (powered_ && mode_ == kContinuousHighResolution && clock_->NowNs() >= ready_ns_) {
        sample_ = static_cast<uint16_t>(std::lround(kDefaultAmbientLux * kCountsPerLux));
    }
    const uint8_t byte = read_index_ == 0u ? static_cast<uint8_t>(sample_ >> 8u)
                                           : static_cast<uint8_t>(sample_);
    ++read_index_;
    return byte;
}

void Bh1750LightSensor::SaveState(StateWriter& w) {
    w.Write("bh1750_ready_ns", ready_ns_);
    w.Write("bh1750_sample", sample_);
    w.Write("bh1750_mode", mode_);
    w.Write("bh1750_read_index", read_index_);
    w.Write("bh1750_powered", powered_);
}

void Bh1750LightSensor::RestoreState(StateReader& r) {
    r.Read("bh1750_ready_ns", ready_ns_);
    r.Read("bh1750_sample", sample_);
    r.Read("bh1750_mode", mode_);
    r.Read("bh1750_read_index", read_index_);
    r.Read("bh1750_powered", powered_);
}

}
