#include "au1300_clock_model.h"

#include "au1300_model_state.h"
#include "au1300_model_host.h"
#include "au1300_dma_model.h"
#include "au1300_gpic_model.h"
#include "au1300_uart_model.h"

#include <algorithm>

namespace cerfcore {

Au1300ClockModel::Au1300ClockModel(Au1300ModelHost& host) : host_(host) {
    Reset(true);
}

void Au1300ClockModel::Reset(bool hardware) {
    if (!hardware) {
        regs_[0x3Cu] &= ~0x60u;
        regs_[0x64u] = 0x14u;
        return;
    }
    toy_ = {};
    rtc_ = {};
    next_event_ = UINT64_MAX;
    for (uint32_t offset : {0u, 4u, 8u, 12u, 16u, 24u, 28u, 44u, 64u, 68u,
                            72u, 76u, 80u, 84u, 88u}) {
        regs_[offset] = 0u;
    }
    regs_[0x14u] = 0x120u;
    regs_[0x20u] = 0u;
    regs_[0x24u] = 0x40903409u;
    regs_[0x28u] = 0x0A0E6287u;
    regs_[0x3Cu] = 2u;
    regs_[0x60u] = 0x42u;
    regs_[0x64u] = 0x14u;
    regs_[0x68u] = 0x3Fu;
}

uint32_t Au1300ClockModel::CounterNow(const Counter& counter, uint32_t enable) {
    const uint32_t required = enable | (1u << 8u);
    if ((regs_.at(0x14u) & required) != required) return counter.value;
    const uint64_t delta = host_.Cycles() - counter.anchor;
    const uint64_t ticks = (delta * 32768u + counter.phase) /
                           (CpuHz() * (counter.trim + 1ull));
    return counter.value + static_cast<uint32_t>(ticks);
}

void Au1300ClockModel::AnchorCounter(Counter& counter, uint32_t enable) {
    const uint64_t now = host_.Cycles();
    const uint32_t required = enable | (1u << 8u);
    if ((regs_.at(0x14u) & required) == required) {
        const uint64_t units = (now - counter.anchor) * 32768u + counter.phase;
        const uint64_t period = CpuHz() * (counter.trim + 1ull);
        counter.value += static_cast<uint32_t>(units / period);
        counter.ticks += units / period;
        counter.phase = units % period;
    }
    counter.anchor = now;
}

void Au1300ClockModel::SetTrim(Counter& counter, uint32_t enable, uint32_t value) {
    AnchorCounter(counter, enable);
    const uint32_t trim = value & 0xFFFFu;
    counter.phase = counter.phase * (trim + 1ull) / (counter.trim + 1ull);
    counter.trim = trim;
}

uint64_t Au1300ClockModel::NextTick(const Counter& counter, uint32_t enable) {
    const uint32_t required = enable | (1u << 8u);
    if ((regs_.at(0x14u) & required) != required) return UINT64_MAX;
    const uint64_t now = host_.Cycles();
    const uint64_t period = CpuHz() * (counter.trim + 1ull);
    const uint64_t units = (now - counter.anchor) * 32768u + counter.phase;
    return now + (period - units % period + 32767u) / 32768u;
}

void Au1300ClockModel::Arm() {
    next_event_ = std::min(NextTick(toy_, 1u << 11u), NextTick(rtc_, 1u << 13u));
    host_.SetSysDeadline(next_event_);
}

void Au1300ClockModel::TickCounter(Counter& counter, uint32_t enable, uint32_t source, uint32_t match_offset) {
    AnchorCounter(counter, enable);
    const uint64_t advanced = counter.ticks - counter.delivered;
    if (advanced == 0u) return;
    const uint32_t previous = counter.value - static_cast<uint32_t>(advanced);
    counter.delivered = counter.ticks;
    host_.Gpic().Pulse(source);
    for (uint32_t i = 0u; i < 3u; ++i) {
        const uint32_t distance = regs_.at(match_offset + 4u * i) - previous;
        const uint64_t ticks = distance != 0u ? distance : 0x100000000ull;
        if (advanced >= ticks) host_.Gpic().Pulse(source + 1u + i);
    }
}

void Au1300ClockModel::Tick() {
    if (host_.Cycles() < next_event_) return;
    TickCounter(toy_, 1u << 11u, 80u, 8u);
    TickCounter(rtc_, 1u << 13u, 84u, 0x4Cu);
    Arm();
}

uint32_t Au1300ClockModel::Read(uint32_t offset) {
    if (offset == 0x40u) return CounterNow(toy_, 1u << 11u);
    if (offset == 0x58u) return CounterNow(rtc_, 1u << 13u);
    const auto it = regs_.find(offset);
    if (it == regs_.end()) host_.Fail("SYS read offset=%X pc=%08X", offset, host_.Pc());
    return it->second;
}

void Au1300ClockModel::Write(uint32_t offset, uint32_t value) {
    Tick();
    const uint32_t previous_divider = PeripheralDivider();
    if (offset == 0x3Cu) {
        for (uint32_t index = 0u; index < 4u; ++index) host_.Uart(index).Advance();
        host_.Dma().Tick();
    }
    if (!regs_.contains(offset) || offset == 0x40u || offset == 0x58u) {
        host_.Fail("SYS write offset=%X value=%08X pc=%08X", offset, value, host_.Pc());
    }
    if (offset == 0x14u && (value & (1u << 14u)) != 0u) {
        host_.Fail("external TOY/RTC counter clock bypass is unmodeled");
    }
    if (offset == 0u) SetTrim(toy_, 1u << 11u, value);
    if (offset == 0x44u) SetTrim(rtc_, 1u << 13u, value);
    if (offset == 4u) {
        AnchorCounter(toy_, 1u << 11u);
        toy_.value = value;
    }
    if (offset == 0x48u) {
        AnchorCounter(rtc_, 1u << 13u);
        rtc_.value = value;
    }
    if (offset == 0x60u) {
        const uint64_t previous = regs_.at(offset) & 0x7Fu;
        value &= 0x7Fu;
        if (value == 0u) host_.Fail("zero CPUPLL is unmodeled");
        AnchorCounter(toy_, 1u << 11u);
        AnchorCounter(rtc_, 1u << 13u);
        toy_.phase = toy_.phase * value / previous;
        rtc_.phase = rtc_.phase * value / previous;
    }
    if (offset == 0x14u) {
        AnchorCounter(toy_, 1u << 11u);
        AnchorCounter(rtc_, 1u << 13u);
        constexpr uint32_t writable = (1u << 14u) | (1u << 13u) | (1u << 12u) |
                                      (1u << 11u) | (1u << 10u) | (1u << 8u);
        regs_[offset] = (value & writable) | 0x20u;
    } else {
        regs_[offset] = value;
    }
    if (offset == 0x3Cu) {
        host_.Dma().ClockChanged(previous_divider / 2u, BusDivider());
        for (uint32_t index = 0u; index < 4u; ++index) {
            host_.Uart(index).ClockChanged(previous_divider, PeripheralDivider());
        }
    }
    if (offset == 0x60u) host_.SetCpuHz(CpuHz());
    if (offset == 0x20u || offset == 0x24u || offset == 0x28u || offset == 0x64u || offset == 0x68u) {
        for (auto& listener : derived_clock_listeners_) listener();
    }
    Arm();
}

void Au1300ClockModel::RegisterDerivedClockListener(std::function<void()> listener) {
    derived_clock_listeners_.push_back(std::move(listener));
}

uint32_t Au1300ClockModel::PeripheralDivider() const {
    return 2u * BusDivider();
}

uint32_t Au1300ClockModel::BusDivider() const {
    return (regs_.at(0x3Cu) & 3u) + 2u;
}

uint64_t Au1300ClockModel::CpuHz() const {
    return 12000000ull * (regs_.at(0x60u) & 0x7Fu);
}

void Au1300ClockModel::SaveState(StateWriter& w) {
    au1300_state::SaveRegisters(w, regs_);
    for (auto* counter : {&toy_, &rtc_}) {
        w.Write("value", counter->value);
        w.Write("trim", counter->trim);
        w.Write("anchor", counter->anchor);
        w.Write("phase", counter->phase);
        w.Write("ticks", counter->ticks);
        w.Write("delivered", counter->delivered);
    }
}

void Au1300ClockModel::RestoreState(StateReader& r) {
    au1300_state::RestoreRegisters(r, regs_);
    for (auto* counter : {&toy_, &rtc_}) {
        r.Read("value", counter->value);
        r.Read("trim", counter->trim);
        r.Read("anchor", counter->anchor);
        r.Read("phase", counter->phase);
        r.Read("ticks", counter->ticks);
        r.Read("delivered", counter->delivered);
    }
    if ((regs_.at(0x14u) & 0x4000u) != 0u) r.Reject("Au1300 counter bypass");
}

void Au1300ClockModel::PostRestore() {
    host_.SetCpuHz(CpuHz());
    Arm();
}

}
