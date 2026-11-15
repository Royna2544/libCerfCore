#pragma once

#include "small_page_nand_flash.h"

#include <cstdint>
#include <string>
#include <vector>

namespace cerfcore {

class VolatileSmallPageNandStore final : public SmallPageNandStore {
public:
    VolatileSmallPageNandStore(CerfEmulator& emu, uint32_t blocks);
    void Open(const std::string& path);
    void ReadPage(uint32_t page, uint8_t* destination) override;
    void WritePage(uint32_t page, const uint8_t* source) override;
    void EraseBlock(uint32_t block) override;
    void SaveState(StateWriter& w);
    void RestoreState(StateReader& r);

private:
    size_t PageOffset(uint32_t page) const;
    CerfEmulator& emu_;
    uint32_t blocks_;
    std::vector<uint8_t> cells_;
};

}
