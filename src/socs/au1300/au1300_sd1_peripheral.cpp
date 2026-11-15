#include "au1300_id.h"
#include "au1300_gpic_model.h"
#include "au1300_model_host.h"
#include "au1300_dma_model.h"
#include "../../peripherals/mmc/mmc_card.h"
#include "../../peripherals/peripheral_dispatcher.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/peripherals/peripheral_base.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>

#include <array>
#include <deque>
#include <vector>

namespace cerfcore {
namespace {

class Au1300Sd1Peripheral : public Peripheral {
public:
    using Peripheral::Peripheral;
    bool ShouldRegister() override { return emu_.Get<BoardContext>().GetSocId() == SocId::Au1300; }
    uint32_t MmioBase() const override { return 0x10601000u; }
    uint32_t MmioSize() const override { return 0x3Cu; }
    void OnReady() override {
        card_ = emu_.TryGet<MmcCard>();
        emu_.Get<PeripheralDispatcher>().Register(this);
        auto& reset = emu_.Get<GuestCpuReset>();
        reset.RegisterResetListener([this](ResetLineKind) { Reset(); });
        reset.RegisterResetReleaseListener([this] { Publish(); });
    }
    uint8_t ReadByte(uint32_t address) override {
        if (address != MmioBase() + 4u) HaltUnsupportedAccess("SD1 ReadByte", address, 0u);
        return ReadRx();
    }
    uint16_t ReadHalf(uint32_t address) override {
        if (address != MmioBase() + 4u) HaltUnsupportedAccess("SD1 ReadHalf", address, 0u);
        return ReadRx();
    }
    void WriteByte(uint32_t address, uint8_t value) override {
        if (address != MmioBase()) HaltUnsupportedAccess("SD1 WriteByte", address, value);
        WriteTx(value);
    }
    void WriteHalf(uint32_t address, uint16_t value) override {
        if (address != MmioBase()) HaltUnsupportedAccess("SD1 WriteHalf", address, value);
        WriteTx(static_cast<uint8_t>(value));
    }
    uint32_t ReadWord(uint32_t address) override {
        const auto offset = address - MmioBase();
        switch (offset) {
        case 4u: {
            uint32_t value = 0u;
            for (uint32_t i = 0u; i < PortBytes(); ++i) value |= static_cast<uint32_t>(ReadRx()) << (i * 8u);
            return value;
        }
        case 8u: case 0xCu: case 0x10u: case 0x14u: case 0x20u: case 0x24u:
        case 0x28u: case 0x2Cu: case 0x30u: case 0x38u:
            return registers_[offset / 4u];
        case 0x34u: registers_[8u] &= ~2u; return registers_[13u];
        case 0x18u: return Status();
        case 0x1Cu: return 0u;
        default: HaltUnsupportedAccess("SD1 ReadWord", address, 0u);
        }
    }
    void WriteWord(uint32_t address, uint32_t value) override {
        const auto offset = address - MmioBase();
        switch (offset) {
        case 0u:
            for (uint32_t i = 0u; i < PortBytes(); ++i) WriteTx(static_cast<uint8_t>(value >> (i * 8u)));
            return;
        case 8u: {
            const auto divider = (value & 0x200u) != 0u ? value & 0x1FFu : registers_[2u] & 0x1FFu;
            registers_[2u] = (value & 0xFF7FFE00u) | divider;
            break;
        }
        case 0xCu:
            if ((value & ~3u) != 0u) HaltUnsupportedAccess("SD1 enable", address, value);
            if (value == 1u) Reset();
            registers_[3u] = value;
            break;
        case 0x10u:
            if ((value & ~0x71Fu) != 0u) HaltUnsupportedAccess("SD1 config2", address, value);
            registers_[4u] = value & ~2u;
            if ((value & 2u) != 0u) { tx_fifo_.clear(); write_data_.clear(); }
            break;
        case 0x14u: registers_[5u] = value & 0x01FF07FFu; break;
        case 0x18u: registers_[6u] &= ~(value & 0x407F7800u); break;
        case 0x20u:
            if ((value & ~0x00FFFFF3u) != 0u) HaltUnsupportedAccess("SD1 command", address, value);
            StartCommand(value);
            break;
        case 0x24u: registers_[9u] = value; break;
        case 0x38u: registers_[14u] = value & 0x1FFFFFu; break;
        default: HaltUnsupportedAccess("SD1 WriteWord", address, value);
        }
        Publish();
    }
    void SaveState(StateWriter& w) override {
        w.Write("registers", registers_);
        w.Write("mode", mode_);
        w.Write("cursor", cursor_);
        w.Write("multiple", multiple_);
        w.Write("tx_size", static_cast<uint32_t>(tx_fifo_.size()));
        for (auto byte : tx_fifo_) w.Write("tx", byte);
        w.Write("write_size", static_cast<uint32_t>(write_data_.size()));
        w.WriteBytes("write_data", write_data_.data(), write_data_.size());
        w.Write("card", card_ != nullptr);
        if (card_) card_->SaveState(w);
    }
    void RestoreState(StateReader& r) override {
        r.Read("registers", registers_);
        r.Read("mode", mode_);
        r.Read("cursor", cursor_);
        r.Read("multiple", multiple_);
        uint32_t size = 0u;
        r.Read("tx_size", size);
        if (size > 32u) r.Reject("SD1 TX FIFO size");
        tx_fifo_.clear();
        for (uint32_t i = 0u; i < size; ++i) { uint8_t byte = 0u; r.Read("tx", byte); tx_fifo_.push_back(byte); }
        r.Read("write_size", size);
        if (size > 512u) r.Reject("SD1 write block size");
        write_data_.resize(size);
        r.ReadBytes("write_data", write_data_.data(), write_data_.size());
        bool card = false;
        r.Read("card", card);
        if (card != (card_ != nullptr)) r.Reject("SD1 card presence");
        if (card_) card_->RestoreState(r);
        if ((registers_[3u] & ~3u) != 0u || (registers_[4u] & ~0x71Fu) != 0u || mode_ > 2u ||
            (mode_ != 0u && !card_) || (mode_ == 1u && cursor_ > card_->ReadData().size())) {
            r.Reject("Au1300 SD1 register state");
        }
    }
    void PostRestore() override { if (card_) card_->PostRestore(); Publish(); }

private:
    void Reset() {
        registers_ = {};
        tx_fifo_.clear();
        write_data_.clear();
        mode_ = cursor_ = 0u;
        multiple_ = false;
    }
    bool Enabled() const { return registers_[3u] == 3u && (registers_[4u] & 3u) == 1u; }
    uint32_t PortBytes() const { return (registers_[4u] & 0x400u) != 0u ? 4u : 1u; }
    uint32_t Available() const {
        return mode_ == 1u && card_ ? static_cast<uint32_t>(card_->ReadData().size()) - cursor_ : 0u;
    }
    void StartCommand(uint32_t value) {
        registers_[8u] = value & ~3u;
        if ((value & 1u) == 0u) return;
        if (!Enabled()) HaltUnsupportedAccess("SD1 command while disabled", MmioBase() + 0x20u, value);
        const auto index = static_cast<uint8_t>(value >> 8u);
        uint32_t response[4]{};
        const auto result = card_ ? card_->Command(index, registers_[9u], response) : MmcCommandResult::NoResponse;
        if ((value & 0xFF0000u) != 0u && result == MmcCommandResult::NoResponse) {
            registers_[6u] |= 0x20000u;
            return;
        }
        registers_[6u] |= 0x10000u;
        if (result != MmcCommandResult::NoResponse) {
            registers_[8u] |= 2u;
            if (result == MmcCommandResult::Long) {
                for (uint32_t i = 0u; i < 4u; ++i) {
                    registers_[10u + i] = response[i] >> 8u;
                    if (i != 0u) registers_[10u + i] |= response[i - 1u] << 24u;
                }
            } else { registers_[12u] = index; registers_[13u] = response[0u]; }
        }
        if (index == 12u) { mode_ = 0u; registers_[6u] |= 0x40000u; return; }
        const auto type = (value >> 4u) & 15u;
        if (type == 2u || type == 4u || type == 6u) {
            if (!card_ || card_->ReadData().empty()) {
                HaltUnsupportedAccess("SD1 read command without data", MmioBase() + 0x20u, value);
            }
            mode_ = 1u;
            cursor_ = 0u;
            multiple_ = type == 4u;
        } else if (type == 1u || type == 3u || type == 5u) {
            mode_ = 2u;
            multiple_ = type == 3u;
            DrainTx();
        }
    }
    uint8_t ReadRx() {
        if (!Enabled() || Available() == 0u) {
            registers_[6u] |= 0x2000u;
            Publish();
            return 0u;
        }
        const auto byte = card_->ReadData()[cursor_++];
        if (Available() == 0u) {
            card_->NextBlock();
            cursor_ = 0u;
            if (!multiple_ || card_->ReadData().empty()) {
                mode_ = 0u;
                registers_[6u] |= 0x40000u;
            }
        }
        Publish();
        return byte;
    }
    void WriteTx(uint8_t value) {
        if (tx_fifo_.size() == 8u * PortBytes()) {
            registers_[6u] |= 0x1000u;
        } else tx_fifo_.push_back(value);
        DrainTx();
        Publish();
    }
    void DrainTx() {
        while (Enabled() && mode_ == 2u && !tx_fifo_.empty()) {
            write_data_.push_back(tx_fifo_.front());
            tx_fifo_.pop_front();
            if (write_data_.size() == (registers_[5u] & 0x7FFu) + 1u) {
                card_->ReceiveBlock(write_data_.data(), static_cast<uint32_t>(write_data_.size()));
                write_data_.clear();
                registers_[6u] = (registers_[6u] & ~7u) | 2u;
                if (!multiple_) { mode_ = 0u; registers_[6u] |= 0x40000u; }
            }
        }
    }
    uint32_t Status() const {
        auto status = registers_[6u];
        if (Enabled()) {
            status |= 0x40u;
            if (tx_fifo_.empty()) status |= 0x02000000u;
            if (tx_fifo_.size() <= PortBytes()) status |= 0x04000000u;
            if (tx_fifo_.size() <= 4u * PortBytes()) status |= 0x01000000u;
            const auto available = Available();
            if (available != 0u) status |= 0x420u;
            if (available >= 4u * PortBytes()) status |= 0x08000000u;
            if (available >= 7u * PortBytes()) status |= 0x10000000u;
            if (available >= 8u * PortBytes()) status |= 0x20000000u;
            if (mode_ == 2u) status |= 0x20u;
        }
        const auto enabled = registers_[2u];
        if ((enabled & 0x8000u) != 0u && (status & enabled & 0xFF7F7C00u) != 0u) status |= 0x8000u;
        return status;
    }
    void Publish() {
        auto& host = emu_.Get<Au1300ModelHost>();
        host.Gpic().SetLevel(32u, (Status() & 0x8000u) != 0u);
        host.Dma().SetRequest(10u, Enabled() && tx_fifo_.size() <= 4u * PortBytes());
        host.Dma().SetRequest(11u, Enabled() && Available() >= 4u * PortBytes());
    }
    std::array<uint32_t, 15u> registers_{};
    MmcCard* card_ = nullptr;
    std::deque<uint8_t> tx_fifo_;
    std::vector<uint8_t> write_data_;
    uint32_t mode_ = 0u;
    uint32_t cursor_ = 0u;
    bool multiple_ = false;
};

}
REGISTER_SERVICE(Au1300Sd1Peripheral);
}
