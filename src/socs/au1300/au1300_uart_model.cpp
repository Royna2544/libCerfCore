#include "au1300_uart_model.h"

#include "au1300_model_state.h"
#include "au1300_model_host.h"
#include "au1300_clock_model.h"
#include "au1300_dma_model.h"
#include "au1300_gpic_model.h"

#include <algorithm>

namespace cerfcore {

Au1300UartModel::Au1300UartModel(Au1300ModelHost& host, uint32_t index) : host_(host), index_(index) {
    Reset();
}

void Au1300UartModel::Reset() {
    for (uint32_t offset : {8u, 16u, 20u, 24u, 36u, 40u, 256u, 260u, 264u}) regs_[offset] = 0u;
    regs_[0x28u] = 1u;
    fifo_.clear();
    receive_fifo_.clear();
    shift_.reset();
    overrun_ = false;
    break_received_ = false;
    break_pending_ = false;
    break_deadline_ = UINT64_MAX;
    receive_deadline_ = UINT64_MAX;
    modem_status_ = external_modem_;
    next_event_ = 0u;
}

uint64_t Au1300UartModel::FrameCycles() const {
    const uint32_t lcr = regs_.at(0x14u);
    const uint32_t data_bits = 5u + (lcr & 3u);
    const uint32_t parity = (lcr & 8u) != 0u ? 1u : 0u;
    const uint32_t stop_halves = (lcr & 4u) == 0u ? 2u : (data_bits == 5u ? 3u : 4u);
    const uint32_t frame_halves = 2u * (1u + data_bits + parity) + stop_halves;
    const uint32_t divisor = regs_.at(0x28u) & 0xFFFFu;
    if (divisor == 0u) host_.Fail("UART%u zero clock divider during transfer", index_);
    return static_cast<uint64_t>(frame_halves) * 8u * divisor * host_.Sys().PeripheralDivider();
}

uint64_t Au1300UartModel::TimeoutCycle() const {
    return receive_deadline_;
}

void Au1300UartModel::ClockChanged(uint64_t previous, uint64_t current) {
    const uint64_t now = regs_.at(0x100u) == 2u ? pause_anchor_ : host_.Cycles();
    const auto rescale = [=](uint64_t deadline) {
        if (deadline == UINT64_MAX || deadline <= now) return deadline;
        return now + ((deadline - now) * current + previous - 1u) / previous;
    };
    if (shift_) deadline_ = rescale(deadline_);
    receive_deadline_ = rescale(receive_deadline_);
    break_deadline_ = rescale(break_deadline_);
    Publish();
}

uint32_t Au1300UartModel::ReceiveThreshold() const {
    constexpr uint32_t thresholds[] = {1u, 4u, 8u, 14u};
    return (regs_.at(0x10u) & 1u) != 0u ? thresholds[(regs_.at(0x10u) >> 6u) & 3u] : 1u;
}

uint32_t Au1300UartModel::TransmitThreshold() const {
    return (regs_.at(0x10u) & 1u) != 0u ? ((regs_.at(0x10u) >> 4u) & 3u) * 4u : 0u;
}

bool Au1300UartModel::BreakVisible() const {
    return break_pending_ && (receive_fifo_.empty() ||
           (receive_fifo_.front().errors & 0x10u) != 0u);
}

uint32_t Au1300UartModel::InterruptCause() const {
    const uint32_t enable = regs_.at(8u);
    if ((enable & 4u) != 0u && (overrun_ || BreakVisible() ||
        (!receive_fifo_.empty() && receive_fifo_.front().errors != 0u))) return 6u;
    if ((enable & 1u) != 0u && !receive_fifo_.empty()) {
        const uint64_t now = regs_.at(0x100u) == 2u ? pause_anchor_ : host_.Cycles();
        if (now >= TimeoutCycle()) return 12u;
        if (receive_fifo_.size() >= ReceiveThreshold()) return 4u;
    }
    if ((enable & 2u) != 0u && fifo_.size() <= TransmitThreshold()) return 2u;
    if ((enable & 8u) != 0u && (modem_status_ & 15u) != 0u) return 0u;
    return 1u;
}

void Au1300UartModel::Publish() {
    UpdateModem();
    constexpr uint32_t interrupts[] = {88u, 17u, 25u, 27u};
    const bool working = regs_.at(0x100u) == 3u;
    host_.Gpic().SetLevel(interrupts[index_], working && InterruptCause() != 1u);
    const bool threshold_mode = (regs_.at(0x10u) & 8u) != 0u;
    host_.Dma().SetRequest(index_ * 2u, working && (threshold_mode ?
        fifo_.size() <= TransmitThreshold() : !shift_ && fifo_.empty()));
    host_.Dma().SetRequest(index_ * 2u + 1u, working && (threshold_mode ?
        receive_fifo_.size() >= ReceiveThreshold() : !receive_fifo_.empty()));
    uint64_t next = working && shift_ ? deadline_ : UINT64_MAX;
    if (working && !receive_fifo_.empty() && (regs_.at(8u) & 1u) != 0u &&
        TimeoutCycle() > host_.Cycles()) next = std::min(next, TimeoutCycle());
    if (working) next = std::min(next, break_deadline_);
    next_event_ = next;
    host_.SetUartDeadline(index_, next);
}

void Au1300UartModel::UpdateModem() {
    const uint32_t control = regs_.at(0x18u);
    uint32_t current = external_modem_;
    if ((control & 0x10u) != 0u) {
        const uint32_t rts = (regs_.at(0x24u) & 1u) != 0u ?
            (receive_fifo_.size() < ReceiveThreshold() ? 2u : 0u) : control & 2u;
        current = (rts << 3u) | ((control & 1u) << 5u) |
                  ((control & 4u) << 4u) | ((control & 8u) << 4u);
    }
    const uint32_t disabled = regs_.at(0x104u);
    const uint32_t disabled_status = ((disabled & 1u) << 4u) | ((disabled & 2u) << 6u) |
                                     ((disabled & 4u) << 3u) | ((disabled & 8u) << 3u);
    current &= ~disabled_status;
    const uint32_t changed = (modem_status_ ^ current) & 0xF0u;
    const uint32_t delta = ((changed & 0x10u) >> 4u) | ((changed & 0x20u) >> 4u) |
                           ((changed & 0x80u) >> 4u) | ((changed & modem_status_ & 0x40u) >> 4u);
    modem_status_ = current | (modem_status_ & 15u) | delta;
}

void Au1300UartModel::SetModemInputs(uint32_t value) {
    Advance();
    external_modem_ = value & 0xF0u;
    UpdateModem();
    Start(host_.Cycles());
    Publish();
}

void Au1300UartModel::Start(uint64_t cycle) {
    if (shift_ || fifo_.empty() || regs_.at(0x100u) != 3u) return;
    if ((regs_.at(0x24u) & 1u) != 0u &&
        ((external_modem_ & 0x10u) == 0u || (regs_.at(0x104u) & 1u) != 0u)) return;
    shift_ = fifo_.front();
    fifo_.pop_front();
    deadline_ = cycle + FrameCycles();
}

void Au1300UartModel::Emit(uint8_t value) {
    host_.EmitUartChar(index_, value, line_);
}

void Au1300UartModel::Receive(uint8_t value, uint8_t errors, uint64_t cycle) {
    if (break_pending_) return;
    const uint32_t capacity = (regs_.at(0x10u) & 1u) != 0u ? 16u : 1u;
    if (receive_fifo_.size() == capacity) overrun_ = true;
    else receive_fifo_.push_back({value, errors});
    if ((errors & 0x10u) != 0u) break_pending_ = true;
    receive_deadline_ = cycle + 0x300ull * regs_.at(0x28u) * host_.Sys().PeripheralDivider();
}

void Au1300UartModel::Advance() {
    const uint64_t now = host_.Cycles();
    if (now < next_event_) return;
    if (regs_.at(0x100u) == 3u) {
        while (shift_ && deadline_ <= now) {
            if ((regs_.at(0x14u) & 0x40u) == 0u) {
                if ((regs_.at(0x18u) & 0x10u) != 0u) Receive(*shift_, 0u, deadline_);
                else Emit(*shift_);
            }
            shift_.reset();
            Start(deadline_);
        }
        if (break_deadline_ <= now) {
            Receive(0u, 0x18u, break_deadline_);
            break_received_ = true;
            break_deadline_ = UINT64_MAX;
        }
    }
    Publish();
}

void Au1300UartModel::FlushLine() {
    host_.EmitUartLine(index_, line_);
    line_.clear();
}

uint32_t Au1300UartModel::Read(uint32_t offset) {
    Advance();
    if (offset == 0u) {
        const uint32_t value = receive_fifo_.empty() ? 0u : receive_fifo_.front().value;
        if (!receive_fifo_.empty()) receive_fifo_.pop_front();
        receive_deadline_ = host_.Cycles() + 0x300ull * regs_.at(0x28u) * host_.Sys().PeripheralDivider();
        Publish();
        return value;
    }
    if (offset == 0x1Cu) {
        uint32_t status = (fifo_.size() <= TransmitThreshold() ? 0x20u : 0u) |
                           (!shift_ && fifo_.empty() ? 0x40u : 0u) | (overrun_ ? 2u : 0u);
        if (!receive_fifo_.empty()) status |= 1u | receive_fifo_.front().errors;
        if (BreakVisible()) {
            status |= 0x10u;
            break_pending_ = false;
        }
        for (const auto& received : receive_fifo_) if (received.errors != 0u) status |= 0x80u;
        overrun_ = false;
        if (!receive_fifo_.empty()) receive_fifo_.front().errors = 0u;
        Publish();
        return status;
    }
    if (offset == 0xCu) return InterruptCause();
    if (offset == 0x20u) {
        const uint32_t status = modem_status_;
        modem_status_ &= 0xF0u;
        Publish();
        return status;
    }
    const auto it = regs_.find(offset);
    if (it == regs_.end()) host_.Fail("UART%u read offset=%X pc=%08X", index_, offset, host_.Pc());
    return it->second;
}

void Au1300UartModel::Write(uint32_t offset, uint32_t value) {
    Advance();
    if (offset == 0x108u && (value & 3u) != 0u) {
        host_.Fail("UART%u bidirectional gating and open-drain output are unmodeled", index_);
    }
    const uint32_t previous_divisor = regs_.at(0x28u);
    if (offset != 4u && !regs_.contains(offset)) host_.Fail("UART%u write offset=%X pc=%08X", index_, offset, host_.Pc());
    if (offset < 0x100u && (regs_.at(0x100u) & 2u) == 0u) return;
    if (offset == 4u) {
        if (regs_.at(0x100u) != 3u) host_.Fail("UART%u TX while disabled", index_);
        const size_t capacity = (regs_.at(0x10u) & 1u) != 0u ? 16u : 1u;
        if (fifo_.size() >= capacity) host_.Fail("UART%u TX FIFO overflow", index_);
        const uint32_t mask = (1u << (5u + (regs_.at(0x14u) & 3u))) - 1u;
        fifo_.push_back(static_cast<uint8_t>(value & mask));
        Start(host_.Cycles());
        Publish();
        return;
    }
    if (offset == 0x100u) {
        value &= 3u;
        const uint32_t previous = regs_.at(offset);
        if ((value & 2u) == 0u) Reset();
        else if (previous == 3u && value == 2u) pause_anchor_ = host_.Cycles();
        else if (previous == 2u && value == 3u) {
            const uint64_t elapsed = host_.Cycles() - pause_anchor_;
            if (shift_) deadline_ += elapsed;
            if (receive_deadline_ != UINT64_MAX) receive_deadline_ += elapsed;
            if (break_deadline_ != UINT64_MAX) break_deadline_ += elapsed;
        }
    }
    if (offset == 0x10u) {
        if ((value & 4u) != 0u || ((value ^ regs_.at(offset)) & 1u) != 0u) fifo_.clear();
        if ((value & 2u) != 0u || ((value ^ regs_.at(offset)) & 1u) != 0u) {
            receive_fifo_.clear();
            overrun_ = false;
        }
        value &= 0xF9u;
    }
    if (offset == 8u || offset == 0x104u) value &= 15u;
    if (offset == 0x14u) value &= 0x7Fu;
    if (offset == 0x18u) value &= 0x1Fu;
    if (offset == 0x24u) value &= 1u;
    if (offset == 0x108u) value &= 7u;
    if (offset == 0x28u) {
        value &= 0xFFFFu;
        if (value == 0u) host_.Fail("UART%u zero clock divider is unmodeled", index_);
    }
    regs_[offset] = value;
    if (offset == 0x28u) ClockChanged(previous_divisor, value);
    if (offset == 0x18u || offset == 0x104u) UpdateModem();
    const bool loop_break = (regs_.at(0x18u) & 0x10u) != 0u && (regs_.at(0x14u) & 0x40u) != 0u;
    if (!loop_break) {
        break_received_ = false;
        break_deadline_ = UINT64_MAX;
    } else if (!break_received_ && break_deadline_ == UINT64_MAX && regs_.at(0x100u) == 3u) {
        break_deadline_ = host_.Cycles() + FrameCycles();
    }
    Start(host_.Cycles());
    Publish();
}

void Au1300UartModel::SaveState(StateWriter& w) {
    au1300_state::SaveRegisters(w, regs_);
    w.Write("deadline_", deadline_);
    w.Write("next_event_", next_event_);
    w.Write("pause_anchor_", pause_anchor_);
    w.Write("receive_deadline_", receive_deadline_);
    w.Write("break_deadline_", break_deadline_);
    w.Write("modem_status_", modem_status_);
    w.Write("external_modem_", external_modem_);
    w.Write("overrun_", overrun_);
    w.Write("break_received_", break_received_);
    w.Write("break_pending_", break_pending_);
    au1300_state::SaveBytes(w, fifo_);
    w.Write("rx_count", static_cast<uint32_t>(receive_fifo_.size()));
    for (const auto& received : receive_fifo_) {
        w.Write("rx_value", received.value);
        w.Write("rx_errors", received.errors);
    }
    w.Write("shift_present", shift_.has_value());
    if (shift_) w.Write("shift_value", *shift_);
}

void Au1300UartModel::RestoreState(StateReader& r) {
    au1300_state::RestoreRegisters(r, regs_);
    r.Read("deadline_", deadline_);
    r.Read("next_event_", next_event_);
    r.Read("pause_anchor_", pause_anchor_);
    r.Read("receive_deadline_", receive_deadline_);
    r.Read("break_deadline_", break_deadline_);
    r.Read("modem_status_", modem_status_);
    r.Read("external_modem_", external_modem_);
    r.Read("overrun_", overrun_);
    r.Read("break_received_", break_received_);
    r.Read("break_pending_", break_pending_);
    au1300_state::RestoreBytes(r, fifo_, 16u);
    uint32_t count = 0u;
    r.Read("rx_count", count);
    if (count > 16u) r.Reject("Au1300 UART RX FIFO size=%u", count);
    receive_fifo_.clear();
    for (uint32_t i = 0u; i < count; ++i) {
        Received received{};
        r.Read("rx_value", received.value);
        r.Read("rx_errors", received.errors);
        receive_fifo_.push_back(received);
    }
    bool present = false;
    r.Read("shift_present", present);
    shift_.reset();
    if (present) {
        uint8_t value = 0u;
        r.Read("shift_value", value);
        shift_ = value;
    }
    line_.clear();
    if ((regs_.at(0x108u) & 3u) != 0u) r.Reject("Au1300 UART bidirectional mode");
}

void Au1300UartModel::PostRestore() {
    Publish();
}

}
