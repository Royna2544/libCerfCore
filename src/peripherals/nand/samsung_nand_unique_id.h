#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace cerfcore {

class CerfEmulator;
class StateWriter;
class StateReader;

class SamsungNandUniqueId {
public:
    explicit SamsungNandUniqueId(CerfEmulator& emu) : emu_(emu) {}
    void Open(const std::string& path, const std::string& otp_path);
    bool Command(uint8_t value);
    void Address(uint8_t value);
    uint8_t ReadData();
    bool Active() const { return stage_ != Stage::Normal; }
    void Reset();
    void SaveState(StateWriter& w);
    void RestoreState(StateReader& r);

private:
    enum class Stage : uint8_t { Normal, Prefix30, Prefix65, Address, Read };
    [[noreturn]] void Fail(const char* operation, uint32_t value);
    CerfEmulator& emu_;
    std::array<uint8_t, 16u> bytes_{};
    std::array<uint8_t, 32u * 528u> otp_{};
    std::array<uint8_t, 4u> address_{};
    Stage stage_ = Stage::Normal;
    uint32_t address_count_ = 0u;
    uint32_t cursor_ = 0u;
    bool reset_seen_ = false;
};

}
