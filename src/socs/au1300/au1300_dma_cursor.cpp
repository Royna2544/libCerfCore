#include "au1300_dma_model.h"

namespace cerfcore {

const char* Au1300DmaModel::Cursor::Reset(uint32_t address, uint32_t config, bool two_dimensions) {
    pointer = address;
    block_start = address;
    group_start = address;
    movement = (config >> 28u) & 3u;
    burst = 1u << (config >> 30u);
    block = two_dimensions ? (config >> 21u) & 127u : (config >> 14u) & 0x3FFFu;
    stride = two_dimensions ? (config >> 11u) & 1023u : config & 0x3FFFu;
    columns = two_dimensions ? config & 2047u : 0u;
    column = 0u;
    in_block = 0u;
    if (two_dimensions && movement == 1u) return "DMA 2D decrement mode is reserved";
    if (movement == 3u) return "DMA burstable external FIFO addressing is unmodeled";
    if (movement == 1u && burst != 1u) return "DMA decrement requires one datum per grant";
    if (movement >= 2u && stride != 0u) return "DMA FIFO addressing cannot use strides";
    return nullptr;
}

uint32_t Au1300DmaModel::Cursor::Address(uint32_t width) const {
    return movement == 1u ? pointer - width + 1u : pointer;
}

void Au1300DmaModel::Cursor::Advance(uint32_t bytes) {
    if (movement >= 2u) return;
    const bool reverse = movement == 1u;
    pointer = reverse ? pointer - bytes : pointer + bytes;
    in_block += bytes;
    if (stride == 0u || block == 0u || in_block < block) return;
    in_block -= block;
    ++column;
    if (columns != 0u && column == columns) {
        group_start += block;
        block_start = group_start;
        column = 0u;
    } else {
        block_start = reverse ? block_start - stride : block_start + stride;
    }
    pointer = reverse ? block_start - in_block : block_start + in_block;
}

}
