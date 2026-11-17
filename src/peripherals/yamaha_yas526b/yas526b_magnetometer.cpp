#include "yas526b_magnetometer.h"

#include "../../boards/inavi_gn2010/inavi_gn2010_family_id.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/host/host_motion_feed.h>
#include <cerfcore/state/state_stream.h>

#include <algorithm>
#include <cmath>

namespace cerfcore {

REGISTER_SERVICE(Yas526bMagnetometer);

namespace {

constexpr uint8_t  kRegisterMask        = 0xC0u;
constexpr uint8_t  kValueMask           = 0x3Fu;
constexpr uint8_t  kMeasureRegister     = 0x00u;
constexpr uint8_t  kOffsetRegister      = 0x40u;
constexpr uint8_t  kConfigRegister      = 0x80u;
constexpr uint8_t  kControlRegister     = 0xC0u;
constexpr uint8_t  kOffsetSecondAxisBit = 0x20u;
constexpr uint8_t  kCoilDriveMask       = 0x0Fu;
constexpr uint8_t  kCalibrationSelect   = 0x20u;
constexpr uint8_t  kMeasureNormal       = 0u;
constexpr uint8_t  kMeasureOffset       = 1u;
constexpr uint16_t kOffsetResult        = 16u;
constexpr double   kZeroFieldCount      = 640.0;
constexpr double   kCountsPerMicroTesla = 4.0;
constexpr double   kMinCount            = 2.0;
constexpr double   kMaxCount            = 0x4FF;

uint16_t FieldCount(double micro_tesla) {
    return static_cast<uint16_t>(std::lround(
        std::clamp(kZeroFieldCount + micro_tesla * kCountsPerMicroTesla, kMinCount, kMaxCount)));
}

}

bool Yas526bMagnetometer::ShouldRegister() {
    return emu_.Get<BoardContext>().GetBoardFamilyId() == BoardFamilyId::InaviGn2010;
}

void Yas526bMagnetometer::OnReady() {
    feed_ = &emu_.Get<HostMotionFeed>();
}

Yas526bMagnetometer::Part& Yas526bMagnetometer::PartFor(uint8_t slave_addr) {
    return slave_addr == kXyAddress ? xy_ : z_;
}

void Yas526bMagnetometer::Pack(uint16_t value, uint8_t* out) {
    out[0] = static_cast<uint8_t>((value >> 4) & 0x7Fu);
    out[1] = static_cast<uint8_t>((value & 0xFu) << 4);
}

void Yas526bMagnetometer::TxnStart(uint8_t slave_addr) {
    PartFor(slave_addr).read_index = 0u;
}

void Yas526bMagnetometer::TxnWriteByte(uint8_t slave_addr, uint8_t byte) {
    const uint8_t reg   = byte & kRegisterMask;
    const uint8_t value = byte & kValueMask;
    if (reg == kMeasureRegister) {
        Measure(slave_addr, value);
        return;
    }
    if (reg == kOffsetRegister) {
        const bool second = slave_addr == kXyAddress && (value & kOffsetSecondAxisBit) != 0u;
        PartFor(slave_addr).offset[second ? 1u : 0u] =
            static_cast<uint8_t>(value & (slave_addr == kXyAddress ? 0x1Fu : kValueMask));
        return;
    }
    if (reg == kControlRegister && value == 0u) return;
    if (reg == kConfigRegister && (value <= kCoilDriveMask || value == kCalibrationSelect)) {
        PartFor(slave_addr).config = value;
        return;
    }
    emu_.Get<Fatal>().Die("YAS526B: register 0x%02X write 0x%02X at address 0x%02X is not modeled",
                          reg, value, slave_addr);
}

void Yas526bMagnetometer::Measure(uint8_t slave_addr, uint8_t mode) {
    if (mode != kMeasureNormal && mode != kMeasureOffset) {
        emu_.Get<Fatal>().Die("YAS526B: measurement command %u at address 0x%02X is not modeled",
                              mode, slave_addr);
    }
    const HostMotionFeed::Vector field = feed_->MagneticFieldMicroTesla();
    Part& part = PartFor(slave_addr);
    if (slave_addr == kXyAddress) {
        const uint16_t first  = mode == kMeasureOffset ? kOffsetResult : FieldCount(field[1]);
        const uint16_t second = mode == kMeasureOffset ? kOffsetResult : FieldCount(field[0]);
        Pack(first, &part.result[0]);
        Pack(second, &part.result[2]);
        return;
    }
    Pack(mode == kMeasureOffset ? kOffsetResult : FieldCount(field[2]), &part.result[0]);
}

uint8_t Yas526bMagnetometer::TxnReadByte(uint8_t slave_addr) {
    Part& part = PartFor(slave_addr);
    const size_t length = part.result.size();
    if (part.read_index >= length) {
        emu_.Get<Fatal>().Die("YAS526B: read of byte %u from address 0x%02X is past its result",
                              part.read_index, slave_addr);
    }
    const uint8_t index = part.read_index++;
    return part.config == kCalibrationSelect ? 0u : part.result[index];
}

void Yas526bMagnetometer::SaveState(StateWriter& w) {
    for (Part* part : {&xy_, &z_}) {
        w.Write("yas526b_result", part->result);
        w.Write("yas526b_offset", part->offset);
        w.Write("yas526b_config", part->config);
        w.Write("yas526b_read_index", part->read_index);
    }
}

void Yas526bMagnetometer::RestoreState(StateReader& r) {
    for (Part* part : {&xy_, &z_}) {
        r.Read("yas526b_result", part->result);
        r.Read("yas526b_offset", part->offset);
        r.Read("yas526b_config", part->config);
        r.Read("yas526b_read_index", part->read_index);
    }
}

}
