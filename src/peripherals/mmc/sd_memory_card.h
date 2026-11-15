#pragma once

#include "mmc_card.h"
#include <cerfcore/core/sd_card_cid.h>

#include <array>
#include <string>

namespace cerfcore {

class SdMemoryCard : public MmcCard {
public:
    using MmcCard::MmcCard;
    void OnReady() override;
    MmcCommandResult Command(uint8_t index, uint32_t argument, uint32_t response[4]) override;
    const std::vector<uint8_t>& ReadData() const override { return data_; }
    void NextBlock() override;
    void EndDataPhase() override;
    void ReceiveBlock(const uint8_t* data, uint32_t bytes) override;
    void Reset() override;
    void SaveState(StateWriter& w) override;
    void RestoreState(StateReader& r) override;

protected:
    virtual std::string ImagePath() const = 0;
    virtual SdCardCid CardCid() const = 0;

private:
    uint32_t Status(uint32_t state) const;
    MmcCommandResult Application(uint8_t index, uint32_t argument, uint32_t response[4]);
    void BuildCsd(uint32_t response[4]);
    void ReadBlock();
    bool ValidAddress(uint32_t address, uint32_t response[4]);
    std::vector<uint8_t> cells_;
    std::vector<uint8_t> data_;
    uint32_t state_ = 0u;
    uint32_t next_address_ = 0u;
    uint32_t erase_start_ = 0u;
    uint32_t erase_end_ = 0u;
    uint32_t written_blocks_ = 0u;
    uint32_t bus_width_ = 0u;
    bool app_ = false;
    bool multi_read_ = false;
    bool multi_write_ = false;
    bool memory_read_ = false;
};

}
