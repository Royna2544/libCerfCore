#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>

namespace cerfcore {

class Au1300ModelHost;
class StateWriter;
class StateReader;

class Au1300UartModel {
public:
    Au1300UartModel(Au1300ModelHost& host, uint32_t index);
    uint32_t Read(uint32_t offset);
    void Write(uint32_t offset, uint32_t value);
    void Advance();
    void FlushLine();
    void SetModemInputs(uint32_t value);
    void ClockChanged(uint64_t previous, uint64_t current);
    void Reset();

    void SaveState(StateWriter& w);
    void RestoreState(StateReader& r);
    void PostRestore();

private:
    uint64_t FrameCycles() const;
    uint64_t TimeoutCycle() const;
    uint32_t ReceiveThreshold() const;
    uint32_t TransmitThreshold() const;
    uint32_t InterruptCause() const;
    bool BreakVisible() const;
    void Publish();
    void UpdateModem();
    void Receive(uint8_t value, uint8_t errors, uint64_t cycle);
    void Start(uint64_t cycle);
    void Emit(uint8_t value);
    Au1300ModelHost& host_;
    uint32_t index_;
    struct Received {
        uint8_t value;
        uint8_t errors;
    };
    std::map<uint32_t, uint32_t> regs_;
    std::deque<uint8_t> fifo_;
    std::deque<Received> receive_fifo_;
    std::optional<uint8_t> shift_;
    uint64_t deadline_ = 0u;
    uint64_t next_event_ = 0u;
    uint64_t pause_anchor_ = 0u;
    uint64_t receive_deadline_ = UINT64_MAX;
    uint64_t break_deadline_ = UINT64_MAX;
    uint32_t modem_status_ = 0u;
    uint32_t external_modem_ = 0u;
    bool overrun_ = false;
    bool break_received_ = false;
    bool break_pending_ = false;
    std::string line_;
};

}
