#pragma once

#include <cerfcore/state/state_stream.h>

#include <cstdint>
#include <deque>
#include <map>

namespace cerfcore::au1300_state {

inline void SaveRegisters(StateWriter& w, const std::map<uint32_t, uint32_t>& registers) {
    for (const auto& [offset, value] : registers) {
        w.Write("offset", offset);
        w.Write("register", value);
    }
}

inline void RestoreRegisters(StateReader& r, std::map<uint32_t, uint32_t>& registers) {
    for (auto& [expected, value] : registers) {
        uint32_t offset = 0u;
        r.Read("offset", offset);
        if (offset != expected) r.Reject("Au1300 register offset=%X expected=%X", offset, expected);
        r.Read("register", value);
    }
}

inline void SaveBytes(StateWriter& w, const std::deque<uint8_t>& fifo) {
    w.Write("fifo_size", static_cast<uint32_t>(fifo.size()));
    for (uint8_t byte : fifo) w.Write("fifo_byte", byte);
}

inline void RestoreBytes(StateReader& r, std::deque<uint8_t>& fifo, uint32_t capacity) {
    uint32_t size = 0u;
    r.Read("fifo_size", size);
    if (size > capacity) r.Reject("Au1300 FIFO size=%u capacity=%u", size, capacity);
    fifo.clear();
    for (uint32_t i = 0u; i < size; ++i) {
        uint8_t byte = 0u;
        r.Read("fifo_byte", byte);
        fifo.push_back(byte);
    }
}

}
