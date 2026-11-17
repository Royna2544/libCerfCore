#pragma once

#include <cerfcore/platform/host_file.h>

#include <array>
#include <cstdint>
#include <map>
#include <string>

namespace cerfcore {

class StateReader;
class StateWriter;

// Input image with guest writes and erases held in a snapshot overlay, or, when opened
// persistent, written straight to the file.
class VolatileSectorImage {
public:
    static constexpr uint32_t kSectorBytes = 512u;
    bool Open(const std::string& path, bool persistent = false);
    bool Persistent() const { return persistent_; }
    uint64_t Size() const { return size_; }
    bool Read(uint64_t sector, uint8_t* destination);
    bool Write(uint64_t sector, const uint8_t* source);
    bool Erase(uint64_t first, uint64_t end);
    void SaveState(StateWriter& writer) const;
    void RestoreState(StateReader& reader);

private:
    host_platform::HostFile file_;
    uint64_t size_ = 0u;
    uint64_t identity_ = 0u;
    bool persistent_ = false;
    std::map<uint64_t, std::array<uint8_t, kSectorBytes>> writes_;
    std::map<uint64_t, uint64_t> erases_;
};

}
