#include "volatile_sector_image.h"

#include <cerfcore/state/state_stream.h>

#include <algorithm>
#include <cstring>
#include <vector>

namespace cerfcore {

bool VolatileSectorImage::Open(const std::string& path, bool persistent) {
    file_.Close();
    writes_.clear();
    erases_.clear();
    persistent_ = persistent;
    identity_   = 0u;
    const auto mode = persistent ? host_platform::FileMode::kReadWriteExisting
                                 : host_platform::FileMode::kReadExisting;
    if (!file_.Open(path, mode) || !file_.Size(size_) ||
        size_ < kSectorBytes || size_ % kSectorBytes != 0u) return false;
    if (persistent) return true;
    // FNV-1a-64 identifies the complete base for snapshot compatibility. It is
    // not an authenticity check. Only the overlay, not the base, is serialized.
    identity_ = 0xCBF29CE484222325ull;
    std::vector<uint8_t> buffer(1024u * 1024u);
    for (uint64_t offset = 0u; offset < size_;) {
        const auto count = static_cast<uint32_t>(std::min<uint64_t>(buffer.size(), size_ - offset));
        uint32_t done = 0u;
        if (!file_.Read(buffer.data(), count, done) || done != count) return false;
        for (uint32_t i = 0u; i < done; ++i) identity_ = (identity_ ^ buffer[i]) * 0x100000001B3ull;
        offset += done;
    }
    return true;
}

bool VolatileSectorImage::Read(uint64_t sector, uint8_t* destination) {
    if (sector >= size_ / kSectorBytes) return false;
    const auto written = writes_.find(sector);
    if (written != writes_.end()) {
        std::memcpy(destination, written->second.data(), kSectorBytes);
        return true;
    }
    auto erased = erases_.upper_bound(sector);
    if (erased != erases_.begin() && sector < std::prev(erased)->second) {
        std::memset(destination, 0, kSectorBytes);
        return true;
    }
    uint32_t done = 0u;
    return file_.SeekTo(sector * kSectorBytes) && file_.Read(destination, kSectorBytes, done) && done == kSectorBytes;
}

bool VolatileSectorImage::Write(uint64_t sector, const uint8_t* source) {
    if (sector >= size_ / kSectorBytes) return false;
    if (persistent_) {
        uint32_t done = 0u;
        return file_.SeekTo(sector * kSectorBytes) && file_.Write(source, kSectorBytes, done) &&
               done == kSectorBytes;
    }
    std::memcpy(writes_[sector].data(), source, kSectorBytes);
    return true;
}

bool VolatileSectorImage::Erase(uint64_t first, uint64_t end) {
    if (first >= end || end > size_ / kSectorBytes) return false;
    if (persistent_) return file_.ZeroRange(first * kSectorBytes, end * kSectorBytes);
    writes_.erase(writes_.lower_bound(first), writes_.lower_bound(end));
    auto next = erases_.lower_bound(first);
    if (next != erases_.begin() && std::prev(next)->second >= first) --next;
    while (next != erases_.end() && next->first <= end) {
        first = std::min(first, next->first);
        end = std::max(end, next->second);
        next = erases_.erase(next);
    }
    erases_.emplace(first, end);
    return true;
}

void VolatileSectorImage::SaveState(StateWriter& w) const {
    w.Write("image_size", size_);
    w.Write("image_identity", identity_);
    w.Write("image_persistent", persistent_);
    w.Write("erases_count", static_cast<uint64_t>(erases_.size()));
    for (const auto& [first, end] : erases_) {
        w.Write("erase_first", first);
        w.Write("erase_end", end);
    }
    w.Write("writes_count", static_cast<uint64_t>(writes_.size()));
    for (const auto& [sector, bytes] : writes_) {
        w.Write("write_sector", sector);
        w.Write("write_data", bytes);
    }
}

void VolatileSectorImage::RestoreState(StateReader& r) {
    uint64_t size = 0u, identity = 0u, count = 0u;
    r.Read("image_size", size);
    bool persistent = false;
    r.Read("image_identity", identity);
    r.Read("image_persistent", persistent);
    if (persistent != persistent_) r.Reject("SD image writable mode differs from snapshot");
    if (size != size_ || identity != identity_) r.Reject("SD base image differs from snapshot");
    erases_.clear();
    writes_.clear();
    r.Read("erases_count", count);
    for (uint64_t i = 0u; i < count; ++i) {
        uint64_t first = 0u, end = 0u;
        r.Read("erase_first", first);
        r.Read("erase_end", end);
        erases_.emplace(first, end);
    }
    r.Read("writes_count", count);
    for (uint64_t i = 0u; i < count; ++i) {
        uint64_t sector = 0u;
        r.Read("write_sector", sector);
        r.Read("write_data", writes_[sector]);
    }
}

}
