#include "sd_memory_card.h"

#include "emmc_register_encoding.h"
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/core/log.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>

#include <algorithm>
#include <cstring>

namespace cerfcore {

void SdMemoryCard::OnReady() {
    const auto path = ImagePath();
    if (!image_.Open(path, ImageWritable()) || image_.Size() < 4096u || image_.Size() > (32ull << 30u)) {
        emu_.Get<Fatal>().UserError("SD image open/geometry path=%s bytes=%llu", path.c_str(),
                                   static_cast<unsigned long long>(image_.Size()));
    }
    high_capacity_ = image_.Size() > (1ull << 30u);
    if (high_capacity_ && (image_.Size() <= (2ull << 30u) || image_.Size() % (512u * 1024u) != 0u))
        emu_.Get<Fatal>().UserError("SDHC image requires >2GiB,512KiB-aligned capacity");
    uint32_t csd[4]{};
    BuildCsd(csd);
    emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) { Reset(); });
    LOG(Boot, "SD slot=%u input=%s bytes=%llu volatile=%u\n", SlotIndex(), path.c_str(),
        static_cast<unsigned long long>(image_.Size()), image_.Persistent() ? 0u : 1u);
}

void SdMemoryCard::Reset() {
    state_ = 0u;
    next_address_ = erase_start_ = erase_end_ = written_blocks_ = bus_width_ = 0u;
    app_ = multi_read_ = multi_write_ = memory_read_ = false;
    interface_condition_ = false;
    data_.clear();
}

uint32_t SdMemoryCard::Status(uint32_t state) const { return (state << 9u) | 0x100u; }

void SdMemoryCard::BuildCsd(uint32_t response[4]) {
    std::fill_n(response, 4u, 0u);
    const auto set = [response](uint32_t low, uint32_t width, uint32_t value) {
        for (uint32_t i = 0u; i < width; ++i) {
            response[3u - (low + i) / 32u] |= ((value >> i) & 1u) << ((low + i) % 32u);
        }
    };
    // SDA Physical Layer7.10,5.3.3/Table5-16 (CSD2) and5.3.2 (CSD1).
    set(112u, 8u, 0x0Eu);
    set(96u, 8u, 0x32u);
    set(84u, 12u, high_capacity_ ? 0x535u : 0x135u);
    set(80u, 4u, 9u);
    if (high_capacity_) {
        set(126u, 2u, 1u);
        set(48u, 22u, static_cast<uint32_t>(image_.Size() / (512u * 1024u)) - 1u);
        set(39u, 7u, 0x7Fu);
    } else {
        uint32_t multiplier = 0u;
        for (; multiplier < 8u; ++multiplier) {
            const uint32_t unit = 512u << (multiplier + 2u);
            if (image_.Size() % unit == 0u && image_.Size() / unit <= 4096u) break;
        }
        if (multiplier == 8u) emu_.Get<Fatal>().UserError("SD SDSC capacity=%llu", static_cast<unsigned long long>(image_.Size()));
        set(62u, 12u, static_cast<uint32_t>(image_.Size() / (512u << (multiplier + 2u))) - 1u);
        set(47u, 3u, multiplier);
    }
    set(46u, 1u, 1u);
    set(26u, 3u, 2u);
    set(22u, 4u, 9u);
    SdCardCid bytes{};
    for (uint32_t i = 0u; i < 16u; ++i) bytes[i] = static_cast<uint8_t>(response[i / 4u] >> ((3u - i % 4u) * 8u));
    EncodeEmmcCid(bytes, response);
}

MmcCommandResult SdMemoryCard::Command(uint8_t index, uint32_t argument, uint32_t response[4]) {
    const auto before = state_;
    std::fill_n(response, 4u, 0u);
    response[0u] = Status(before);
    const bool application = app_;
    app_ = false;
    if (index == 0u) { Reset(); return MmcCommandResult::NoResponse; }
    if (application) return Application(index, argument, response);
    const uint64_t address = high_capacity_ ? static_cast<uint64_t>(argument) * 512u : argument;
    switch (index) {
    case 1u: case 5u: case 52u: case 53u: return MmcCommandResult::NoResponse;
    case 8u:
        // SDA7.10,4.3.13: R7 echoes VHS and pattern only for supported voltage.
        if (!high_capacity_ || state_ != 0u || ((argument >> 8u) & 15u) != 1u)
            return MmcCommandResult::NoResponse;
        interface_condition_ = true;
        response[0u] = argument & 0xFFFu;
        return MmcCommandResult::Short;
    case 6u:
        if (!high_capacity_ || state_ != 4u) break;
        SwitchFunction(argument);
        return MmcCommandResult::Short;
    case 55u:
        if ((argument >> 16u) != (state_ < 2u ? 0u : 1u)) return MmcCommandResult::NoResponse;
        app_ = true;
        response[0u] |= 0x20u;
        return MmcCommandResult::Short;
    case 2u:
        if (state_ != 1u) break;
        state_ = 2u;
        EncodeEmmcCid(CardCid(), response);
        return MmcCommandResult::Long;
    case 3u:
        if (state_ != 2u && state_ != 3u) break;
        state_ = 3u;
        response[0u] = 0x10000u | (Status(before) & 0xFFFFu);
        return MmcCommandResult::Short;
    case 7u:
        if ((argument >> 16u) == 1u && state_ == 3u) { state_ = 4u; return MmcCommandResult::Short; }
        if ((argument >> 16u) != 1u) { EndDataPhase(); state_ = 3u; return MmcCommandResult::NoResponse; }
        break;
    case 9u: case 10u:
        if ((argument >> 16u) != 1u) return MmcCommandResult::NoResponse;
        if (state_ != 3u) break;
        if (index == 9u) BuildCsd(response);
        else EncodeEmmcCid(CardCid(), response);
        return MmcCommandResult::Long;
    case 12u:
        if (state_ != 5u && state_ != 6u) break;
        EndDataPhase();
        return MmcCommandResult::Short;
    case 13u:
        if ((argument >> 16u) != 1u) return MmcCommandResult::NoResponse;
        return MmcCommandResult::Short;
    case 16u:
        if (state_ != 4u) break;
        if (argument != 512u) response[0u] |= 1u << 29u;
        return MmcCommandResult::Short;
    case 17u: case 18u:
        if (state_ != 4u) break;
        if (!ValidAddress(address, response)) return MmcCommandResult::Short;
        state_ = 5u;
        memory_read_ = true;
        multi_read_ = index == 18u;
        next_address_ = address;
        ReadBlock();
        return MmcCommandResult::Short;
    case 24u: case 25u:
        if (state_ != 4u) break;
        if (!ValidAddress(address, response)) return MmcCommandResult::Short;
        state_ = 6u;
        multi_write_ = index == 25u;
        next_address_ = address;
        return MmcCommandResult::Short;
    case 32u: case 33u:
        if (state_ != 4u) break;
        if (!ValidAddress(address, response)) return MmcCommandResult::Short;
        (index == 32u ? erase_start_ : erase_end_) = address;
        return MmcCommandResult::Short;
    case 38u:
        if (state_ != 4u || erase_end_ < erase_start_) break;
        if (!image_.Erase(erase_start_ / 512u, static_cast<uint64_t>(erase_end_) / 512u + 1u))
            emu_.Get<Fatal>().Die("SD erase range");
        return MmcCommandResult::Short;
    default: break;
    }
    emu_.Get<Fatal>().Die("SD command=%u arg=%08X state=%u", index, argument, state_);
}

MmcCommandResult SdMemoryCard::Application(uint8_t index, uint32_t argument, uint32_t response[4]) {
    response[0u] |= 0x20u;
    if (index == 41u && state_ == 0u) {
        // SDA7.10,4.2.3: SDHC stays busy without CMD8 and host HCS support.
        response[0u] = high_capacity_ ? 0x00FF8000u : 0x80FF8000u;
        if (argument != 0u) {
            if ((argument & 0xFF8000u) == 0u) state_ = 8u;
            else if (!high_capacity_ || (interface_condition_ && (argument & 0x40000000u) != 0u)) {
                state_ = 1u;
                if (high_capacity_) response[0u] |= 0xC0000000u;
            }
        }
        return MmcCommandResult::Short;
    }
    if (state_ != 4u) emu_.Get<Fatal>().Die("SD ACMD=%u state=%u", index, state_);
    switch (index) {
    case 6u:
        if (argument != 0u && argument != 2u) response[0u] |= 1u << 22u;
        else bus_width_ = argument;
        return MmcCommandResult::Short;
    case 23u: case 42u: return MmcCommandResult::Short;
    case 51u: data_ = {static_cast<uint8_t>(high_capacity_ ? 2u : 0u), 0x05u, 0u, 0u, 0u, 0u, 0u, 0u}; break;
    case 13u:
        data_.assign(64u, 0u);
        data_[0u] = static_cast<uint8_t>(bus_width_ << 6u);
        break;
    case 22u:
        data_.resize(4u);
        for (uint32_t i = 0u; i < 4u; ++i) data_[i] = static_cast<uint8_t>(written_blocks_ >> ((3u - i) * 8u));
        break;
    default: emu_.Get<Fatal>().Die("SD ACMD=%u arg=%08X", index, argument);
    }
    state_ = 5u;
    memory_read_ = multi_read_ = false;
    return MmcCommandResult::Short;
}

void SdMemoryCard::SwitchFunction(uint32_t argument) {
    // SDA7.10,4.3.10.4/Table4-13: default-speed-only card, no busy functions.
    data_.assign(64u, 0u);
    data_[1u] = 200u;
    for (uint32_t group = 0u; group < 6u; ++group) {
        data_[12u - group * 2u] = 0x80u;
        data_[13u - group * 2u] = 1u;
        const uint32_t requested = (argument >> (group * 4u)) & 15u;
        if (requested != 0u && requested != 15u) {
            data_[16u - group / 2u] |= static_cast<uint8_t>(15u << ((group & 1u) * 4u));
            data_[1u] = 0u;
        }
    }
    state_ = 5u;
    memory_read_ = multi_read_ = false;
}

bool SdMemoryCard::ValidAddress(uint64_t address, uint32_t response[4]) {
    if ((address & 511u) != 0u) { response[0u] |= 1u << 30u; return false; }
    if (static_cast<uint64_t>(address) + 512u > image_.Size()) { response[0u] |= 1u << 31u; return false; }
    return true;
}

void SdMemoryCard::ReadBlock() {
    data_.resize(512u);
    if (!image_.Read(next_address_ / 512u, data_.data())) {
        emu_.Get<Fatal>().Die("SD read offset=%llX", static_cast<unsigned long long>(next_address_));
    }
    next_address_ += 512u;
}

void SdMemoryCard::NextBlock() {
    if (memory_read_ && multi_read_) ReadBlock();
    else EndDataPhase();
}

void SdMemoryCard::EndDataPhase() {
    data_.clear();
    multi_read_ = multi_write_ = memory_read_ = false;
    if (state_ == 5u || state_ == 6u) state_ = 4u;
}

void SdMemoryCard::ReceiveBlock(const uint8_t* data, uint32_t bytes) {
    if (state_ != 6u || bytes != 512u || !image_.Write(next_address_ / 512u, data)) {
        emu_.Get<Fatal>().Die("SD write offset=%llX bytes=%u state=%u", static_cast<unsigned long long>(next_address_), bytes, state_);
    }
    next_address_ += bytes;
    ++written_blocks_;
    if (!multi_write_) state_ = 4u;
}

void SdMemoryCard::SaveState(StateWriter& w) {
    w.Write("state", state_);
    w.Write("next_address", next_address_);
    w.Write("erase_start", erase_start_);
    w.Write("erase_end", erase_end_);
    w.Write("written_blocks", written_blocks_);
    w.Write("bus_width", bus_width_);
    w.Write("app", app_);
    w.Write("multi_read", multi_read_);
    w.Write("multi_write", multi_write_);
    w.Write("memory_read", memory_read_);
    w.Write("interface_condition", interface_condition_);
    image_.SaveState(w);
    w.Write("data_size", static_cast<uint32_t>(data_.size()));
    w.WriteBytes("data", data_.data(), data_.size());
}

void SdMemoryCard::RestoreState(StateReader& r) {
    r.Read("state", state_);
    r.Read("next_address", next_address_);
    r.Read("erase_start", erase_start_);
    r.Read("erase_end", erase_end_);
    r.Read("written_blocks", written_blocks_);
    r.Read("bus_width", bus_width_);
    r.Read("app", app_);
    r.Read("multi_read", multi_read_);
    r.Read("multi_write", multi_write_);
    r.Read("memory_read", memory_read_);
    r.Read("interface_condition", interface_condition_);
    image_.RestoreState(r);
    uint32_t size = 0u;
    r.Read("data_size", size);
    if (state_ > 8u || size > 512u || next_address_ > image_.Size() ||
        erase_start_ > image_.Size() - 512u || erase_end_ > image_.Size() - 512u ||
        (bus_width_ != 0u && bus_width_ != 2u)) r.Reject("SD protocol state");
    data_.resize(size);
    r.ReadBytes("data", data_.data(), data_.size());
}

}
