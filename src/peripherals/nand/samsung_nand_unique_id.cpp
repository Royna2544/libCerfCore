#include "samsung_nand_unique_id.h"

#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/core/log.h>
#include <cerfcore/platform/host_file.h>
#include <cerfcore/state/state_stream.h>
#include <algorithm>

namespace cerfcore {

void SamsungNandUniqueId::Open(const std::string& path, const std::string& otp_path) {
    bytes_.fill(0xFFu);
    otp_.fill(0xFFu);
    if (!host_platform::HostPathExists(path)) {
        LOG(SocNand, "Samsung NAND UID: provisional erased factory bytes; no input file\n");
    } else {
    host_platform::HostFile file;
    uint64_t size = 0u;
    uint32_t done = 0u;
    if (!file.Open(path, host_platform::FileMode::kReadExisting) || !file.Size(size) ||
        size != bytes_.size() || !file.Read(bytes_.data(), static_cast<uint32_t>(bytes_.size()), done) ||
        done != bytes_.size()) {
        emu_.Get<Fatal>().UserError("Samsung NAND UID input must contain 16 bytes: %s", path.c_str());
    }
    LOG(SocNand, "Samsung NAND UID input=%s bytes=16\n", path.c_str());
    }
    // NANDUUID.dll C0CA2280 uses the same30/65 read sequence with a page
    // selector. The supplied UID covers page0 only; other cells are provisional
    // erased input unless a separate raw OTP block is supplied.
    std::copy(bytes_.begin(), bytes_.end(), otp_.begin());
    if (host_platform::HostPathExists(otp_path)) {
        host_platform::HostFile file;
        uint64_t size = 0u;
        uint32_t done = 0u;
        if (!file.Open(otp_path, host_platform::FileMode::kReadExisting) || !file.Size(size) ||
            size != otp_.size() || !file.Read(otp_.data(), static_cast<uint32_t>(otp_.size()), done) || done != size)
            emu_.Get<Fatal>().UserError("Samsung NAND OTP input must contain32x528 bytes: %s", otp_path.c_str());
        LOG(SocNand, "Samsung NAND OTP input=%s bytes=%u\n", otp_path.c_str(), done);
    } else LOG(SocNand, "Samsung NAND OTP: page0 UID; other pages provisionally erased\n");
}

void SamsungNandUniqueId::Reset() {
    stage_ = Stage::Normal;
    address_count_ = cursor_ = 0u;
    address_.fill(0u);
    reset_seen_ = false;
}

bool SamsungNandUniqueId::Command(uint8_t value) {
    if (value == 0xFFu) {
        Reset();
        reset_seen_ = true;
        return false;
    }
    switch (stage_) {
    case Stage::Normal:
        if (value == 0x30u && reset_seen_) {
            stage_ = Stage::Prefix30;
            reset_seen_ = false;
            return true;
        }
        reset_seen_ = false;
        return false;
    case Stage::Prefix30:
        if (value != 0x65u) Fail("command after 30", value);
        stage_ = Stage::Prefix65;
        return true;
    case Stage::Prefix65:
        if (value != 0u) Fail("command after 65", value);
        stage_ = Stage::Address;
        return true;
    default: Fail("command during UID read", value);
    }
}

void SamsungNandUniqueId::Address(uint8_t value) {
    if (stage_ != Stage::Address || address_count_ >= 4u) Fail("address", value);
    address_[address_count_++] = value;
    if (address_count_ == 4u) {
        const uint32_t page = address_[1u] | (address_[2u] << 8u) | (address_[3u] << 16u);
        if (page >= 32u) Fail("OTP page", page);
        cursor_ = page * 528u + address_[0u];
        stage_ = Stage::Read;
        if (page != 0u) LOG(SocNand, "Samsung OTP page=%u column=%u\n", page, address_[0u]);
    }
}

uint8_t SamsungNandUniqueId::ReadData() {
    if (stage_ != Stage::Read || cursor_ >= otp_.size()) Fail("data read", cursor_);
    return otp_[cursor_++];
}

void SamsungNandUniqueId::SaveState(StateWriter& w) {
    w.Write("factory_uid", bytes_);
    w.Write("factory_otp", otp_);
    w.Write("uid_address", address_);
    w.Write("uid_stage", stage_);
    w.Write("uid_address_count", address_count_);
    w.Write("uid_cursor", cursor_);
    w.Write("uid_reset_seen", reset_seen_);
}

void SamsungNandUniqueId::RestoreState(StateReader& r) {
    std::array<uint8_t, 16u> bytes;
    r.Read("factory_uid", bytes);
    if (bytes != bytes_) r.Reject("Samsung NAND UID input differs from saved device");
    std::array<uint8_t, 32u * 528u> otp;
    r.Read("factory_otp", otp);
    if (otp != otp_) r.Reject("Samsung NAND OTP input differs from saved device");
    r.Read("uid_address", address_);
    r.Read("uid_stage", stage_);
    r.Read("uid_address_count", address_count_);
    r.Read("uid_cursor", cursor_);
    r.Read("uid_reset_seen", reset_seen_);
    if (stage_ > Stage::Read || address_count_ > 4u || cursor_ > otp_.size()) r.Reject("Samsung NAND UID read state");
}

void SamsungNandUniqueId::Fail(const char* operation, uint32_t value) {
    emu_.Get<Fatal>().Die("Samsung NAND UID %s: %X", operation, value);
}

}
