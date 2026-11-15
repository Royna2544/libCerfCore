#include "au1300_dma_model.h"

#include "au1300_model_state.h"
#include "au1300_model_host.h"
#include "au1300_clock_model.h"
#include "au1300_gpic_model.h"
#include "au1300_uart_model.h"

#include <algorithm>
#include <cerfengine/byte_order.h>

namespace cerfcore {

Au1300DmaModel::Au1300DmaModel(Au1300ModelHost& host) : host_(host) {
    Reset();
}

void Au1300DmaModel::Reset() {
    for (auto& channel : channels_) channel = {};
    for (auto& channel : channels_) channel.regs[5] = 1u;
    requests_ = {};
    requests_[31u] = true;
    edges_ = {};
    round_robin_ = {};
    config_ = throttle_ = high_grants_ = fetch_round_robin_ = 0u;
    interrupt_enable_ = 0xFFFFu;
    throttle_epoch_ = 0u;
    throttle_phase_ = 0u;
    next_event_ = UINT64_MAX;
    pumping_ = false;
}

uint32_t Au1300DmaModel::Read(uint32_t offset) {
    if (offset == 0x1000u) return config_;
    if (offset == 0x1008u) return throttle_;
    if (offset == 0x100Cu) return interrupt_enable_;
    if (offset == 0x1004u) {
        uint32_t pending = 0u;
        for (uint32_t i = 0u; i < channels_.size(); ++i) pending |= channels_[i].regs[4] << i;
        return pending;
    }
    if (offset >= 0x1000u || (offset & 0xFFu) > 0x18u || (offset & 3u) != 0u) {
        host_.Fail("DMA read offset=%X pc=%08X", offset, host_.Pc());
    }
    if ((offset & 0xFFu) == 0xCu) host_.Fail("DMA read from write-only doorbell");
    return channels_[offset >> 8u].regs[(offset & 0xFFu) / 4u];
}

void Au1300DmaModel::Write(uint32_t offset, uint32_t value) {
    Tick();
    if (offset == 0x1000u) config_ = value & 7u;
    else if (offset == 0x1008u) {
        throttle_ = value & 0x8000FFFFu;
        throttle_epoch_ = host_.Cycles();
        throttle_phase_ = 0u;
        edges_[30u] = {};
        for (auto& channel : channels_) {
            channel.edge_baseline[30u] = {};
            for (uint32_t side = 0u; side < 2u; ++side) {
                if (((channel.descriptor[0] >> (side != 0u ? 20u : 25u)) & 31u) == 30u) {
                    channel.edge_seen[side] = 0u;
                }
            }
        }
    } else if (offset == 0x100Cu) interrupt_enable_ = value & 0xFFFFu;
    else {
        if (offset >= 0x1000u || (offset & 0xFFu) > 0x14u || (offset & 3u) != 0u) {
            host_.Fail("DMA write offset=%X value=%08X", offset, value);
        }
        auto& channel = channels_[offset >> 8u];
        const uint32_t reg = (offset & 0xFFu) / 4u;
        if (reg == 0u) {
            if ((channel.regs[0] & 1u) == 0u && (value & 1u) != 0u) channel.edge_baseline = edges_;
            channel.regs[0] = value & 0x3EFu;
            if ((value & 1u) == 0u) channel.regs[5] |= 1u;
            else channel.regs[5] &= ~1u;
        } else if (reg == 1u) channel.regs[1] = value & ~31u;
        else if (reg == 2u) channel.regs[2] = value & ~3u;
        else if (reg == 3u) channel.regs[5] |= 4u;
        else if (reg == 4u) channel.regs[4] = value & 1u;
        else if ((channel.regs[5] & 1u) != 0u) {
            channel.regs[5] &= ~(value & 6u);
            if ((value & 2u) != 0u) channel.fifo.clear();
        }
    }
    Tick();
    Publish();
}

void Au1300DmaModel::SetRequest(uint32_t source, bool level) {
    if (source >= 30u) host_.Fail("DMA peripheral request ID=%u", source);
    if (requests_[source] != level) {
        ++edges_[source][level ? 0u : 1u];
        requests_[source] = level;
        host_.SetDmaDeadline(host_.Cycles());
    }
}

bool Au1300DmaModel::Requested(Channel& channel, bool destination) {
    const uint32_t command = channel.descriptor[0];
    const uint32_t id = (command >> (destination ? 20u : 25u)) & 31u;
    const uint32_t shift = destination ? 6u : 8u;
    const uint32_t polarity = (channel.regs[0] >> shift) & 1u;
    if (id == 30u) {
        if ((throttle_ & 0x80000000u) == 0u) return false;
        auto& seen = channel.edge_seen[destination ? 1u : 0u];
        if (edges_[30u][0u] == seen) return false;
        ++seen;
        return true;
    }
    if ((channel.regs[0] & (1u << (shift + 1u))) == 0u) return requests_[id] != (polarity != 0u);
    auto& seen = channel.edge_seen[destination ? 1u : 0u];
    if (seen == edges_[id][polarity]) return false;
    seen = edges_[id][polarity];
    return true;
}

bool Au1300DmaModel::Fetch(Channel& channel) {
    if ((channel.regs[5] & 2u) != 0u) return true;
    if ((channel.regs[5] & 4u) == 0u) return false;
    channel.regs[5] &= ~4u;
    for (uint32_t i = 0u; i < channel.descriptor.size(); ++i) {
        channel.descriptor[i] = host_.ReadPhysical(channel.regs[1] + 4u * i, 4u);
    }
    auto& d = channel.descriptor;
    if ((d[0] & 0x80000000u) == 0u) return false;
    if ((d[1] & 0xFF000000u) != 0u && ((d[0] >> 13u) & 3u) != 2u) {
        host_.Fail("DMA requires a physical address above 32 bits");
    }
    const uint32_t type = (d[0] >> 13u) & 3u;
    if (type == 3u || ((d[0] >> 18u) & 3u) == 3u || ((d[0] >> 16u) & 3u) == 3u) {
        host_.Fail("DMA reserved descriptor encoding command=%08X", d[0]);
    }
    if ((d[0] & 0x400u) != 0u &&
        (((d[0] >> 18u) & 3u) != 2u || ((d[0] >> 16u) & 3u) != 2u)) {
        host_.Fail("DMA 2D stride requires word source and destination widths");
    }
    channel.regs[5] |= 2u;
    channel.regs[6] = type == 2u ? ((d[7] >> 28u) & 3u) + 1u : d[1] & 0x3FFFFFu;
    channel.source_remaining = channel.regs[6];
    channel.literal_offset = 0u;
    channel.compare_value = 0u;
    channel.source_grant = 0u;
    channel.destination_grant = 0u;
    for (uint32_t side = 0u; side < 2u; ++side) {
        const uint32_t id = (d[0] >> (side != 0u ? 20u : 25u)) & 31u;
        const uint32_t polarity = (channel.regs[0] >> (side != 0u ? 6u : 8u)) & 1u;
        channel.edge_seen[side] = channel.edge_baseline[id][polarity];
    }
    const char* source_error = type == 0u ?
        channel.source.Reset(d[2], d[3], (d[0] & 0x400u) != 0u) :
        channel.source.Reset(0u, 0u, false);
    if (source_error) host_.Fail("%s", source_error);
    if (type != 2u) {
        const char* destination_error = channel.destination.Reset(d[4], d[5], (d[0] & 0x400u) != 0u);
        if (destination_error) host_.Fail("%s", destination_error);
    }
    if (type == 1u && (channel.regs[6] > 8u || ((d[0] >> 18u) & 3u) != 2u)) {
        host_.Fail("DMA literal requires word source width and at most 8 bytes");
    }
    if (type == 2u && (d[1] & 0xF0000000u) != 0u) host_.Fail("DMA branch source above 32 bits");
    if (type == 2u && ((d[0] >> 16u) & 3u) != 2u) host_.Fail("DMA compare destination must be word width");
    return true;
}

bool Au1300DmaModel::FetchNext() {
    const bool weighted = (config_ & 4u) != 0u;
    for (uint32_t slot = 0u; slot < channels_.size(); ++slot) {
        const uint32_t index = weighted ? slot : (fetch_round_robin_ + slot) % channels_.size();
        auto& channel = channels_[index];
        if ((channel.regs[0] & 1u) == 0u || (channel.regs[5] & 2u) != 0u ||
            (channel.regs[5] & 4u) == 0u) continue;
        Fetch(channel);
        fetch_round_robin_ = (index + 1u) % channels_.size();
        return true;
    }
    return false;
}

void Au1300DmaModel::Complete(Channel& channel, uint32_t next) {
    const auto& d = channel.descriptor;
    if ((d[0] & 0x10u) != 0u) channel.regs[2] = channel.regs[1];
    const uint32_t status = d[0] & 3u;
    if ((d[0] & 0x10u) != 0u || status != 0u) {
        uint32_t value = d[6];
        if ((d[0] & 0x10u) == 0u && status == 2u) value = d[0] & 0x7FFFFFFFu;
        if ((d[0] & 0x10u) == 0u && status == 3u) value = channel.regs[6];
        host_.WritePhysical(channel.regs[2], value, 4u);
    }
    if ((d[0] & 4u) != 0u) {
        host_.WritePhysical(channel.regs[1], host_.ReadPhysical(channel.regs[1], 4u) & 0x7FFFFFFFu, 4u);
    }
    if ((d[0] & 0x100u) != 0u) channel.regs[4] = 1u;
    if ((next & 0x08000000u) != 0u) host_.Fail("DMA next-pointer subroutine mode is not grounded");
    channel.regs[1] = (next & 0x07FFFFFFu) << 5u;
    channel.regs[5] = (channel.regs[5] & ~2u) | 4u;
    channel.edge_baseline = edges_;
    Publish();
}

bool Au1300DmaModel::Step(Channel& channel) {
    auto& d = channel.descriptor;
    const uint32_t type = (d[0] >> 13u) & 3u;
    if (type == 2u) {
        if (!Requested(channel, false)) return false;
        const uint32_t width = 1u << ((d[0] >> 18u) & 3u);
        const uint32_t address = d[2] < 0x10000000u ? d[2] + channel.literal_offset : d[2];
        uint32_t value = host_.ReadPhysical(address, width);
        if ((channel.regs[0] & 4u) != 0u) {
            std::array<uint8_t, 8u> bytes{};
            cerfengine::le::PutN(bytes.data(), value, width);
            std::reverse(bytes.begin(), bytes.begin() + width);
            value = static_cast<uint32_t>(cerfengine::le::UN(bytes.data(), width));
        }
        const uint32_t bytes = std::min(width, channel.regs[6]);
        channel.compare_value |= (value & cerfengine::ByteWidthMask(bytes)) << (8u * channel.literal_offset);
        channel.literal_offset += bytes;
        channel.regs[6] -= bytes;
        if (channel.regs[6] == 0u) Complete(channel, (channel.compare_value & d[5]) == d[4] ? d[1] : d[7]);
        return true;
    }
    if (channel.regs[6] == 0u) {
        Complete(channel, d[7]);
        return true;
    }
    const uint32_t sw = 1u << ((d[0] >> 18u) & 3u);
    const uint32_t dw = 1u << ((d[0] >> 16u) & 3u);
    if (channel.source_remaining != 0u && channel.fifo.size() <= 32u - sw &&
        channel.source_grant == 0u && (type == 1u || Requested(channel, false))) {
        channel.source_grant = channel.source.burst;
    }
    if (channel.source_remaining != 0u && channel.fifo.size() <= 32u - sw && channel.source_grant != 0u) {
        for (; channel.source_grant != 0u && channel.source_remaining != 0u &&
             channel.fifo.size() <= 32u - sw;) {
            const uint32_t value = type == 1u ? d[2u + channel.literal_offset / 4u] :
                host_.ReadPhysical(channel.source.Address(sw), sw);
            const uint32_t bytes = std::min(sw, channel.source_remaining);
            for (uint32_t i = 0u; i < bytes; ++i) {
                const uint32_t lane = (channel.regs[0] & 4u) != 0u ? sw - 1u - i : i;
                channel.fifo.push_back(static_cast<uint8_t>(value >> (8u * lane)));
            }
            channel.source.Advance(bytes);
            channel.source_remaining -= bytes;
            channel.literal_offset += bytes;
            --channel.source_grant;
        }
        return true;
    }
    if (channel.fifo.size() < std::min(dw, channel.regs[6])) return false;
    if (channel.destination_grant == 0u) {
        if (!Requested(channel, true)) return false;
        channel.destination_grant = channel.destination.burst;
    }
    for (; channel.destination_grant != 0u && channel.regs[6] != 0u &&
         channel.fifo.size() >= std::min(dw, channel.regs[6]); --channel.destination_grant) {
        const uint32_t bytes = std::min(dw, channel.regs[6]);
        uint32_t value = 0u;
        for (uint32_t i = 0u; i < bytes; ++i) {
            const uint32_t lane = (channel.regs[0] & 2u) != 0u ? bytes - 1u - i : i;
            value |= static_cast<uint32_t>(channel.fifo.front()) << (8u * lane);
            channel.fifo.pop_front();
        }
        host_.WritePhysical(channel.destination.Address(bytes), value, bytes);
        channel.destination.Advance(bytes);
        channel.regs[6] -= bytes;
    }
    return true;
}

void Au1300DmaModel::Publish() {
    host_.Gpic().SetLevel(75u, (Read(0x1004u) & interrupt_enable_) != 0u);
}

void Au1300DmaModel::AdvanceThrottle() {
    const uint64_t now = host_.Cycles();
    if ((throttle_ & 0x80000000u) != 0u) {
        const uint64_t period = ((throttle_ & 0xFFFFu) + 1ull) * host_.Sys().BusDivider();
        const uint64_t units = now - throttle_epoch_ + throttle_phase_;
        edges_[30u][0u] += units / period;
        throttle_phase_ = units % period;
    }
    throttle_epoch_ = now;
}

void Au1300DmaModel::ArmThrottle() {
    uint64_t next = UINT64_MAX;
    if ((throttle_ & 0x80000000u) != 0u) {
        for (const auto& channel : channels_) {
            if ((channel.regs[0] & 1u) == 0u || (channel.regs[5] & 2u) == 0u) continue;
            const uint32_t command = channel.descriptor[0];
            if ((((command >> 25u) & 31u) == 30u && channel.source_remaining != 0u) ||
                (((command >> 20u) & 31u) == 30u && channel.regs[6] != 0u)) {
                const uint64_t period = ((throttle_ & 0xFFFFu) + 1ull) * host_.Sys().BusDivider();
                next = host_.Cycles() + period - throttle_phase_;
                break;
            }
        }
    }
    next_event_ = next;
    host_.SetDmaDeadline(next);
}

void Au1300DmaModel::ClockChanged(uint32_t previous, uint32_t current) {
    throttle_phase_ = throttle_phase_ * current / previous;
    ArmThrottle();
}

void Au1300DmaModel::Tick() {
    if (pumping_) return;
    pumping_ = true;
    AdvanceThrottle();
    uint32_t grant = 0u;
    for (; grant < 4096u; ++grant) {
        const bool fetched = FetchNext();
        bool progress = false;
        const uint32_t first_pool = high_grants_ < 8u ? 1u : 0u;
        for (uint32_t pass = 0u; pass < 2u && !progress; ++pass) {
            const uint32_t pool = first_pool ^ pass;
            const bool weighted = (config_ & (1u << pool)) != 0u;
            for (uint32_t slot = 0u; slot < channels_.size(); ++slot) {
                const uint32_t index = weighted ? slot : (round_robin_[pool] + slot) % channels_.size();
                auto& channel = channels_[index];
                if ((channel.regs[0] & 1u) == 0u || (channel.regs[5] & 2u) == 0u ||
                    ((channel.descriptor[0] >> 15u) & 1u) != pool || !Step(channel)) continue;
                round_robin_[pool] = (index + 1u) % channels_.size();
                high_grants_ = pool != 0u ? high_grants_ + 1u : 0u;
                progress = true;
                break;
            }
        }
        if (!progress && !fetched) break;
    }
    pumping_ = false;
    ArmThrottle();
    if (grant == 4096u) {
        next_event_ = std::min(next_event_, host_.Cycles() + 1u);
        host_.SetDmaDeadline(next_event_);
    }
}

void Au1300DmaModel::SaveState(StateWriter& w) {
    w.Write("requests_", requests_);
    w.Write("edges_", edges_);
    w.Write("round_robin_", round_robin_);
    w.Write("config_", config_);
    w.Write("throttle_", throttle_);
    w.Write("interrupt_enable_", interrupt_enable_);
    w.Write("high_grants_", high_grants_);
    w.Write("fetch_round_robin_", fetch_round_robin_);
    w.Write("throttle_epoch_", throttle_epoch_);
    w.Write("throttle_phase_", throttle_phase_);
    for (auto& channel : channels_) {
        w.Write("regs", channel.regs);
        w.Write("descriptor", channel.descriptor);
        w.Write("edge_seen", channel.edge_seen);
        w.Write("edge_baseline", channel.edge_baseline);
        w.Write("source_remaining", channel.source_remaining);
        w.Write("literal_offset", channel.literal_offset);
        w.Write("compare_value", channel.compare_value);
        w.Write("source_grant", channel.source_grant);
        w.Write("destination_grant", channel.destination_grant);
        for (auto* cursor : {&channel.source, &channel.destination}) {
            w.Write("pointer", cursor->pointer);
            w.Write("block_start", cursor->block_start);
            w.Write("group_start", cursor->group_start);
            w.Write("movement", cursor->movement);
            w.Write("block", cursor->block);
            w.Write("stride", cursor->stride);
            w.Write("columns", cursor->columns);
            w.Write("column", cursor->column);
            w.Write("in_block", cursor->in_block);
            w.Write("burst", cursor->burst);
        }
        au1300_state::SaveBytes(w, channel.fifo);
    }
}

void Au1300DmaModel::RestoreState(StateReader& r) {
    r.Read("requests_", requests_);
    r.Read("edges_", edges_);
    r.Read("round_robin_", round_robin_);
    r.Read("config_", config_);
    r.Read("throttle_", throttle_);
    r.Read("interrupt_enable_", interrupt_enable_);
    r.Read("high_grants_", high_grants_);
    r.Read("fetch_round_robin_", fetch_round_robin_);
    r.Read("throttle_epoch_", throttle_epoch_);
    r.Read("throttle_phase_", throttle_phase_);
    for (auto& channel : channels_) {
        r.Read("regs", channel.regs);
        r.Read("descriptor", channel.descriptor);
        r.Read("edge_seen", channel.edge_seen);
        r.Read("edge_baseline", channel.edge_baseline);
        r.Read("source_remaining", channel.source_remaining);
        r.Read("literal_offset", channel.literal_offset);
        r.Read("compare_value", channel.compare_value);
        r.Read("source_grant", channel.source_grant);
        r.Read("destination_grant", channel.destination_grant);
        for (auto* cursor : {&channel.source, &channel.destination}) {
            r.Read("pointer", cursor->pointer);
            r.Read("block_start", cursor->block_start);
            r.Read("group_start", cursor->group_start);
            r.Read("movement", cursor->movement);
            r.Read("block", cursor->block);
            r.Read("stride", cursor->stride);
            r.Read("columns", cursor->columns);
            r.Read("column", cursor->column);
            r.Read("in_block", cursor->in_block);
            r.Read("burst", cursor->burst);
        }
        au1300_state::RestoreBytes(r, channel.fifo, 32u);
    }
    pumping_ = false;
}

void Au1300DmaModel::PostRestore() {
    Publish();
    host_.SetDmaDeadline(host_.Cycles());
}

}
