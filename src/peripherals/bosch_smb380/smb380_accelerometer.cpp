#include "smb380_accelerometer.h"

#include "../../boards/inavi_gn2010/inavi_gn2010_family_id.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/host/host_motion_feed.h>
#include <cerfcore/state/state_stream.h>

#include <algorithm>
#include <cmath>

namespace cerfcore {

REGISTER_SERVICE(Smb380Accelerometer);

namespace {

constexpr uint8_t kChipIdRegister   = 0x00u;
constexpr uint8_t kChipId           = 0x02u;
constexpr uint8_t kAccXLsbRegister  = 0x02u;
constexpr uint8_t kLastDataRegister = 0x09u;
constexpr uint8_t kCtrl0Register    = 0x0Au;
constexpr uint8_t kSoftResetMask    = 0x02u;
constexpr uint8_t kSelfTestMask     = 0x0Cu;
constexpr uint8_t kRangeRegister    = 0x14u;
constexpr uint8_t kRangeMask        = 0x18u;
constexpr uint8_t kRangePos         = 3u;
constexpr uint8_t kNewDataFlag      = 0x01u;

}

bool Smb380Accelerometer::ShouldRegister() {
    return emu_.Get<BoardContext>().GetBoardFamilyId() == BoardFamilyId::InaviGn2010;
}

void Smb380Accelerometer::OnReady() {
    feed_ = &emu_.Get<HostMotionFeed>();
    Reset();
}

void Smb380Accelerometer::Reset() {
    regs_.fill(0u);
    regs_[kChipIdRegister] = kChipId;
}

void Smb380Accelerometer::TxnStart(uint8_t) {
    have_pointer_ = false;
}

void Smb380Accelerometer::TxnWriteByte(uint8_t, uint8_t byte) {
    if (!have_pointer_) {
        if (byte >= kRegisterCount) {
            emu_.Get<Fatal>().Die("SMB380: register pointer 0x%02X is outside the register map", byte);
        }
        pointer_      = byte;
        have_pointer_ = true;
        return;
    }
    WriteRegister(pointer_, byte);
    pointer_ = static_cast<uint8_t>(pointer_ + 1u);
}

void Smb380Accelerometer::WriteRegister(uint8_t reg, uint8_t value) {
    if (reg >= kRegisterCount || reg <= kLastDataRegister) {
        emu_.Get<Fatal>().Die("SMB380: write 0x%02X to read-only register 0x%02X", value, reg);
    }
    if (reg == kCtrl0Register && (value & kSelfTestMask) != 0u) {
        emu_.Get<Fatal>().Die("SMB380: self-test request 0x%02X is not modeled", value);
    }
    if (reg == kCtrl0Register && (value & kSoftResetMask) != 0u) {
        Reset();
        return;
    }
    regs_[reg] = value;
}

void Smb380Accelerometer::LatchSample() {
    const HostMotionFeed::Vector g = feed_->AccelerationG();
    const uint32_t range_code = (regs_[kRangeRegister] & kRangeMask) >> kRangePos;
    if (range_code > 2u) {
        emu_.Get<Fatal>().Die("SMB380: reserved range code %u", range_code);
    }
    const double counts_per_g = 256.0 / static_cast<double>(1u << range_code);
    for (size_t axis = 0; axis < 3u; ++axis) {
        const long counts = std::lround(std::clamp(g[axis] * counts_per_g, -512.0, 511.0));
        const uint16_t raw = static_cast<uint16_t>(counts) & 0x3FFu;
        sample_[axis * 2u]      = static_cast<uint8_t>(((raw & 3u) << 6) | kNewDataFlag);
        sample_[axis * 2u + 1u] = static_cast<uint8_t>(raw >> 2);
    }
}

uint8_t Smb380Accelerometer::TxnReadByte(uint8_t) {
    if (pointer_ >= kRegisterCount) {
        emu_.Get<Fatal>().Die("SMB380: read past the register map at 0x%02X", pointer_);
    }
    const uint8_t reg = pointer_;
    pointer_          = static_cast<uint8_t>(pointer_ + 1u);
    if (reg == kAccXLsbRegister) LatchSample();
    const uint8_t value = reg >= kAccXLsbRegister && reg < kAccXLsbRegister + 6u
                              ? sample_[reg - kAccXLsbRegister] : regs_[reg];
    return value;
}

void Smb380Accelerometer::SaveState(StateWriter& w) {
    w.Write("smb380_regs", regs_);
    w.Write("smb380_sample", sample_);
    w.Write("smb380_pointer", pointer_);
    w.Write("smb380_have_pointer", have_pointer_);
}

void Smb380Accelerometer::RestoreState(StateReader& r) {
    r.Read("smb380_regs", regs_);
    r.Read("smb380_sample", sample_);
    r.Read("smb380_pointer", pointer_);
    r.Read("smb380_have_pointer", have_pointer_);
}

}
