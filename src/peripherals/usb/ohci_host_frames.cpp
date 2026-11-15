#include "ohci_host_controller.h"

#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>

#include <algorithm>

namespace cerfcore {

void OhciHostController::SetClocked(bool clocked) {
    Advance();
    clocked_ = clocked;
    Arm();
}

void OhciHostController::BeginFrame() {
    frame_left_ = (registers_[13u] & 0x3FFFu) + 1u;
    frame_toggle_ = registers_[13u] & 0x80000000u;
    const auto previous = registers_[15u];
    registers_[15u] = (previous + 1u) & 0xFFFFu;
    if (((previous ^ registers_[15u]) & 0x8000u) != 0u) registers_[3u] |= 0x20u;
    WriteBus(registers_[6u] + 0x80u, registers_[15u], 4u);
    registers_[3u] |= 4u;
    periodic_done_ = false;
    NonPeriodic(8u, 9u, 2u, 0x10u);
    NonPeriodic(10u, 11u, 4u, 0x20u);
    Publish();
}

void OhciHostController::NonPeriodic(uint32_t head, uint32_t current, uint32_t filled, uint32_t enable) {
    if ((registers_[1u] & enable) == 0u) return;
    if (registers_[current] == 0u && (registers_[2u] & filled) != 0u) {
        registers_[current] = registers_[head];
        registers_[2u] &= ~filled;
    }
    Walk(registers_[current]);
}

void OhciHostController::Walk(uint32_t& current) {
    for (uint32_t visited = 0u; current != 0u; ++visited) {
        if (visited == 4096u) emu_.Get<Fatal>().Die("OHCI endpoint list cycle at %08X", current);
        const auto flags = ReadBus(current, 4u);
        const auto tail = ReadBus(current + 4u, 4u) & ~15u;
        const auto head = ReadBus(current + 8u, 4u);
        if ((flags & 0x4000u) == 0u && (head & 1u) == 0u && (head & ~15u) != tail) {
            emu_.Get<Fatal>().Die("OHCI active ED=%08X TD=%08X flags=%08X", current, head, flags);
        }
        current = ReadBus(current + 12u, 4u) & ~15u;
    }
}

void OhciHostController::Periodic() {
    periodic_done_ = true;
    if ((registers_[1u] & 4u) == 0u) return;
    registers_[7u] = ReadBus(registers_[6u] + (registers_[15u] & 31u) * 4u, 4u) & ~15u;
    Walk(registers_[7u]);
}

void OhciHostController::Advance() {
    const auto now = bits_.CountAt(clock_->Cycles());
    uint32_t elapsed = now - last_bits_;
    last_bits_ = now;
    if (!clocked_ || !Operational()) return;
    while (elapsed != 0u) {
        uint32_t until = frame_left_;
        if (!periodic_done_) {
            const auto start = registers_[16u];
            if (frame_left_ <= start + 1u) { Periodic(); continue; }
            until = std::min(until, frame_left_ - start - 1u);
        }
        const auto step = std::min(elapsed, until);
        elapsed -= step;
        frame_left_ -= step;
        if (frame_left_ == 0u) BeginFrame();
    }
    if (!periodic_done_ && frame_left_ <= registers_[16u] + 1u) Periodic();
}

void OhciHostController::Arm() {
    if (!clocked_ || !Operational()) { clock_->Disarm(event_); return; }
    auto left = frame_left_;
    if (!periodic_done_) {
        left = frame_left_ <= registers_[16u] + 1u ? 0u : frame_left_ - registers_[16u] - 1u;
    }
    clock_->Arm(event_, bits_.CycleOfTick(bits_.TicksSince(clock_->Cycles()) + left));
}

}
