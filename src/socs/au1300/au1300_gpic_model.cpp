#include "au1300_gpic_model.h"

#include "au1300_model_state.h"
#include "au1300_model_host.h"
#include "au1300_clock_model.h"
#include "au1300_dma_model.h"
#include "au1300_uart_model.h"

#include <algorithm>

namespace cerfcore {

Au1300GpicModel::Au1300GpicModel(Au1300ModelHost& host) : host_(host) {
    for (uint32_t base : {0u, 0x20u, 0x40u, 0x60u, 0x80u, 0xA0u}) {
        for (uint32_t bank = 0u; bank < 16u; bank += 4u) regs_[base + bank] = 0u;
    }
    for (uint32_t offset = 0x1000u; offset < 0x1200u; offset += 4u) regs_[offset] = 0u;
}

uint32_t Au1300GpicModel::Read(uint32_t offset) {
    if (offset == 0x30u) {
        uint32_t winner = 0x7Fu;
        uint32_t priority = 4u;
        for (uint32_t source = 0u; source < 128u; ++source) {
            const uint32_t bank = (source / 32u) * 4u;
            const uint32_t bit = 1u << (source & 31u);
            const uint32_t candidate = (regs_.at(0x1000u + source * 4u) >> 2u) & 3u;
            if ((regs_.at(0x20u + bank) & regs_.at(0x40u + bank) & bit) != 0u &&
                candidate < priority) {
                winner = source;
                priority = candidate;
            }
        }
        return winner;
    }
    const auto it = regs_.find(offset);
    if (it == regs_.end()) host_.Fail("GPIC read offset=%X pc=%08X", offset, host_.Pc());
    return it->second;
}

void Au1300GpicModel::Reset() {
    for (auto& [offset, value] : regs_) value = 0u;
}

void Au1300GpicModel::Publish() {
    uint32_t lines = 0u;
    for (uint32_t source = 0u; source < 128u; ++source) {
        const uint32_t bank = (source / 32u) * 4u;
        const uint32_t bit = 1u << (source & 31u);
        const uint32_t config = regs_.at(0x1000u + source * 4u);
        const uint32_t detection = (config >> 4u) & 7u;
        const uint32_t control = config & 3u;
        const bool output = source < 75u && control >= 2u;
        const bool level = output ? control == 3u : levels_[source];
        const bool previous = (regs_.at(bank) & bit) != 0u;
        if (level) regs_[bank] |= bit;
        else regs_[bank] &= ~bit;
        if (control == 1u) regs_[0x80u + bank] |= bit;
        else regs_[0x80u + bank] &= ~bit;
        if (previous != level && (detection == 7u ||
            (detection == 5u && !level) || (detection == 6u && level))) {
            regs_[0x20u + bank] |= bit;
        }
        if (detection == 1u || detection == 2u) {
            if (level == (detection == 2u)) regs_[0x20u + bank] |= bit;
            else regs_[0x20u + bank] &= ~bit;
        }
        if ((regs_.at(0x20u + bank) & regs_.at(0x40u + bank) & bit) == 0u) continue;
        const uint32_t priority = (regs_.at(0x1000u + source * 4u) >> 2u) & 3u;
        lines |= 1u << (10u + priority);
    }
    for (uint32_t request = 0u; request < 2u; ++request) {
        const uint32_t pin = (regs_.at(0x60u) >> (request * 8u)) & 0x7Fu;
        const uint32_t bank = (pin / 32u) * 4u;
        const bool level = (regs_.at(bank) & (1u << (pin & 31u))) != 0u;
        host_.Dma().SetRequest(28u + request, level);
    }
    host_.SetGpicLines(lines);
}

void Au1300GpicModel::SetLevel(uint32_t source, bool level) {
    if (source >= levels_.size()) host_.Fail("GPIC level source=%u", source);
    if (levels_[source] == level) return;
    levels_[source] = level;
    Publish();
}

void Au1300GpicModel::Write(uint32_t offset, uint32_t value) {
    if (offset == 0x60u && ((value & ~0x7F7Fu) != 0u ||
        (value & 0x7Fu) >= 75u || ((value >> 8u) & 0x7Fu) >= 75u)) {
        host_.Fail("unmodeled GPIC GPIO DMA selection=%08X", value);
    }
    const uint32_t group = offset & ~0xFu;
    const uint32_t bank = offset & 0xFu;
    if (group == 0u || group == 0x10u || group == 0x80u || group == 0x90u) {
        const uint32_t control = group == 0u ? 3u : group == 0x10u ? 2u : group == 0x80u ? 1u : 0u;
        for (uint32_t pin = 0u; pin < 32u; ++pin) {
            if ((value & (1u << pin)) == 0u) continue;
            auto& config = regs_.at(0x1000u + (bank * 8u + pin) * 4u);
            config = (config & ~3u) | control;
        }
    } else if (group == 0x50u) {
        regs_[group - 0x10u + bank] &= ~value;
    } else {
        if (!regs_.contains(offset)) host_.Fail("GPIC write offset=%X value=%08X", offset, value);
        if (group == 0x20u) regs_[offset] &= ~value;
        else if (group == 0x40u) regs_[offset] |= value;
        else regs_[offset] = value;
    }
    Publish();
    for (const auto& listener : output_listeners_) listener();
}

bool Au1300GpicModel::DrivenLow(uint32_t pin) const {
    if (pin >= 75u) host_.Fail("GPIC output query on input-only source=%u", pin);
    return (regs_.at(0x1000u + pin * 4u) & 3u) == 2u;
}

void Au1300GpicModel::AddOutputListener(std::function<void()> listener) {
    output_listeners_.push_back(std::move(listener));
}

void Au1300GpicModel::AddPinDeviceState(std::function<void(StateWriter&)> save,
                                        std::function<void(StateReader&)> restore) {
    device_save_.push_back(std::move(save));
    device_restore_.push_back(std::move(restore));
}

void Au1300GpicModel::Pulse(uint32_t source) {
    if (source >= 128u) host_.Fail("GPIC pulse source=%u", source);
    const uint32_t config = regs_.at(0x1000u + source * 4u);
    const uint32_t detection = (config >> 4u) & 7u;
    if (detection == 0u) return;
    if (detection < 5u) host_.Fail("GPIC pulse on a non-edge input: source=%u config=%X", source, config);
    const uint32_t bank = (source / 32u) * 4u;
    regs_[0x20u + bank] |= 1u << (source & 31u);
    Publish();
}

void Au1300GpicModel::SaveState(StateWriter& w) {
    au1300_state::SaveRegisters(w, regs_);
    w.Write("levels", levels_);
    for (const auto& save : device_save_) save(w);
}

void Au1300GpicModel::RestoreState(StateReader& r) {
    au1300_state::RestoreRegisters(r, regs_);
    r.Read("levels", levels_);
    for (const auto& restore : device_restore_) restore(r);
}

void Au1300GpicModel::PostRestore() {
    Publish();
}

}
