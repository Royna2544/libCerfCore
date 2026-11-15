#pragma once

#include <array>
#include <cstdint>
#include <map>

namespace cerfcore {

class Au1300ModelHost;
class StateWriter;
class StateReader;

class Au1300GpicModel {
public:
    explicit Au1300GpicModel(Au1300ModelHost& host);
    void Reset();
    uint32_t Read(uint32_t offset);
    void Write(uint32_t offset, uint32_t value);
    void Pulse(uint32_t source);
    void SetLevel(uint32_t source, bool level);

    void SaveState(StateWriter& w);
    void RestoreState(StateReader& r);
    void PostRestore();

private:
    void Publish();
    Au1300ModelHost& host_;
    std::map<uint32_t, uint32_t> regs_;
    std::array<bool, 128u> levels_{};
};

}
