#include "volatile_small_page_nand_store.h"

#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/core/log.h>
#include <cerfcore/platform/host_file.h>
#include <cerfcore/state/state_stream.h>

#include <algorithm>
#include <cstring>

namespace cerfcore {

VolatileSmallPageNandStore::VolatileSmallPageNandStore(CerfEmulator& emu, uint32_t blocks)
    : emu_(emu), blocks_(blocks) {}

void VolatileSmallPageNandStore::Open(const std::string& path) {
    host_platform::HostFile file;
    if (!file.Open(path, host_platform::FileMode::kReadExisting)) {
        emu_.Get<Fatal>().UserError("NAND input open failed: %s", path.c_str());
    }
    const uint64_t expected = static_cast<uint64_t>(blocks_) * kPagesPerBlock * kPageBytes;
    uint64_t size = 0u;
    if (!file.Size(size) || size != expected || size > UINT32_MAX) {
        emu_.Get<Fatal>().UserError("NAND input size=%llu expected=%llu: %s",
                                   static_cast<unsigned long long>(size),
                                   static_cast<unsigned long long>(expected), path.c_str());
    }
    cells_.resize(static_cast<size_t>(size));
    uint32_t read = 0u;
    if (!file.Read(cells_.data(), static_cast<uint32_t>(size), read) || read != size) {
        emu_.Get<Fatal>().UserError("NAND input short read: %s", path.c_str());
    }
    LOG(SocNand, "NAND input=%s bytes=%llu volatile=1\n", path.c_str(),
        static_cast<unsigned long long>(size));
}

size_t VolatileSmallPageNandStore::PageOffset(uint32_t page) const {
    if (page >= static_cast<uint64_t>(blocks_) * kPagesPerBlock) {
        emu_.Get<Fatal>().Die("NAND store page=%u", page);
    }
    return static_cast<size_t>(page) * kPageBytes;
}

void VolatileSmallPageNandStore::ReadPage(uint32_t page, uint8_t* destination) {
    std::memcpy(destination, cells_.data() + PageOffset(page), kPageBytes);
}

void VolatileSmallPageNandStore::WritePage(uint32_t page, const uint8_t* source) {
    std::memcpy(cells_.data() + PageOffset(page), source, kPageBytes);
}

void VolatileSmallPageNandStore::EraseBlock(uint32_t block) {
    if (block >= blocks_) emu_.Get<Fatal>().Die("NAND store block=%u", block);
    std::fill_n(cells_.data() + PageOffset(block * kPagesPerBlock), kPagesPerBlock * kPageBytes, kErased);
}

void VolatileSmallPageNandStore::SaveState(StateWriter& w) {
    w.WriteBytes("nand_cells", cells_.data(), cells_.size());
}

void VolatileSmallPageNandStore::RestoreState(StateReader& r) {
    r.ReadBytes("nand_cells", cells_.data(), cells_.size());
}

}
