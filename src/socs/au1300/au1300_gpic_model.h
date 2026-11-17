#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <vector>

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

    bool DrivenLow(uint32_t pin) const;
    void AddOutputListener(std::function<void()> listener);
    void AddPinDeviceState(std::function<void(StateWriter&)> save,
                           std::function<void(StateReader&)> restore);

    void SaveState(StateWriter& w);
    void RestoreState(StateReader& r);
    void PostRestore();

private:
    void Publish();
    Au1300ModelHost& host_;
    std::map<uint32_t, uint32_t> regs_;
    std::array<bool, 128u> levels_{};
    std::vector<std::function<void()>> output_listeners_;
    std::vector<std::function<void(StateWriter&)>> device_save_;
    std::vector<std::function<void(StateReader&)>> device_restore_;
};

}
