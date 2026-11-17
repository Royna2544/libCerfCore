#include "au1300_id.h"
#include "au1300_model_host.h"
#include "au1300_gpic_model.h"
#include "au1300_dma_model.h"
#include "../cycle_anchored_counter.h"
#include "../../host/paced_wave_out.h"
#include "../../host/audio_activity_widget.h"
#include "../../peripherals/ac97_codec.h"
#include "../../peripherals/peripheral_dispatcher.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/core/instance_threads.h>
#include <cerfcore/peripherals/peripheral_base.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

namespace cerfcore {
namespace {

constexpr uint32_t kEvents = 0x03003F30u;

class Au1300Ac97Peripheral : public Peripheral, public Ac97FrameSource {
public:
    explicit Au1300Ac97Peripheral(CerfEmulator& emu) : Peripheral(emu), frames_(emu) { Reset(); }
    bool ShouldRegister() override { return emu_.Get<BoardContext>().GetSocId() == SocId::Au1300; }
    uint32_t MmioBase() const override { return 0x10A01000u; }
    uint32_t MmioSize() const override { return 0x30u; }
    void OnReady() override {
        clock_ = &emu_.Get<GuestCycleClock>();
        codec_ = &emu_.Get<Ac97Codec>();
        codec_->AttachFrameSource(this);
        frames_.SetRate(clock_->ClockRate(), {48000u, 1u});
        frames_.Anchor(clock_->Cycles(), 0u);
        pcm_frame_ = FrameNow();
        pcm_event_ = clock_->Add([this] { AdvancePcm(); Publish(); ArmPcm(); });
        output_.Start(emu_, "Au1300 AC97", 0u, 2u, 16u);
        emu_.Get<AudioActivityWidget>().NotePresent();
        clock_->RegisterRateListener([this] {
            AdvancePcm();
            const auto now = clock_->Cycles();
            const auto phase = frames_.TickPhaseAt(now);
            frame_base_ += frames_.TicksSince(now);
            frames_.SetRate(clock_->ClockRate(), {48000u, 1u});
            frames_.AnchorAt(now, 0u, phase);
            ArmPcm();
        });
        emu_.Get<PeripheralDispatcher>().Register(this);
        auto& reset = emu_.Get<GuestCpuReset>();
        reset.RegisterResetListener([this](ResetLineKind kind) {
            codec_->LinkStopped(FrameNow());
            Reset();
            clock_->Disarm(pcm_event_);
            output_.StopAudioOut();
            output_active_ = false;
            linked_ = false;
            if (kind == ResetLineKind::Rtc) codec_->ColdReset();
        });
        reset.RegisterResetReleaseListener([this] { Publish(); ArmPcm(); });
    }
    void OnShutdown() override {
        if (clock_ && pcm_event_) clock_->Disarm(pcm_event_);
        output_.Stop();
    }
    void WriteHalf(uint32_t address, uint16_t value) override {
        if (address != MmioBase() + 0x1Cu) HaltUnsupportedAccess("AC97 WriteHalf", address, value);
        WriteWord(address, value);
    }
    uint32_t ReadWord(uint32_t address) override {
        AdvancePcm();
        const auto offset = address - MmioBase();
        switch (offset) {
        case 0u: case 4u: case 8u: case 0xCu: case 0x18u: case 0x20u: case 0x24u:
            return registers_[offset / 4u];
        case 0x10u: return 0u;
        case 0x14u: return Status();
        default: HaltUnsupportedAccess("ReadWord", address, 0u);
        }
    }
    void WriteWord(uint32_t address, uint32_t value) override {
        AdvancePcm();
        const auto offset = address - MmioBase();
        switch (offset) {
        case 0u:
            if ((value & ~0x37u) != 0u || (value != 0u && value != 0x24u)) {
                HaltUnsupportedAccess("PSC1 protocol/clock selection", address, value);
            }
            registers_[0u] = value;
            break;
        case 4u:
            if (value != 0u && value != 2u && value != 3u) HaltUnsupportedAccess("PSC1 enable", address, value);
            if (value == 0u) {
                output_.StopAudioOut();
                output_active_ = false;
                codec_->LinkStopped(FrameNow());
                linked_ = false;
                const auto selection = registers_[0u];
                Reset();
                registers_[0u] = selection;
            }
            registers_[1u] = value;
            break;
        case 8u:
            if ((value & (1u << 26u)) != 0u) {
                const auto length = (value >> 21u) & 15u;
                if (length < 3u || length > 9u) HaltUnsupportedAccess("AC97 data length", address, value);
            }
            registers_[2u] = value;
            break;
        case 0xCu: registers_[3u] = value & kEvents; break;
        case 0x10u:
            if ((value & ~0x67u) != 0u) HaltUnsupportedAccess("AC97 transfer start", address, value);
            if ((value & 0x10u) != 0u) HaltUnsupportedAccess("AC97 receive start", address, value);
            if ((value & 4u) != 0u) tx_fifo_.clear();
            if ((value & 1u) != 0u) {
                if (!linked_ || ((registers_[2u] >> 11u) & 0x3FFu) != 3u) {
                    HaltUnsupportedAccess("AC97 TX slot setup", address, registers_[2u]);
                }
                tx_active_ = true;
                tx_stopping_ = false;
                pcm_frame_ = FrameNow();
            }
            if ((value & 2u) != 0u) { tx_stopping_ = true; if (tx_fifo_.empty()) StopTx(); }
            break;
        case 0x1Cu:
            if (tx_fifo_.size() == 16u) registers_[6u] |= 0x200u;
            else tx_fifo_.push_back(value);
            break;
        case 0x18u: registers_[6u] &= ~(value & kEvents); break;
        case 0x20u:
            if ((value & ~0x03FFFFFFu) != 0u) HaltUnsupportedAccess("AC97 codec command", address, value);
            registers_[8u] = value;
            pending_ = true;
            break;
        case 0x24u:
            if ((value & ~3u) != 0u) HaltUnsupportedAccess("AC97 reset", address, value);
            registers_[9u] = value;
            if ((value & 2u) != 0u) codec_->ColdReset();
            if ((value & 1u) != 0u) codec_->WarmReset();
            break;
        default: HaltUnsupportedAccess("WriteWord", address, value);
        }
        RefreshLink();
        if (pending_ && linked_) ExecuteCommand();
        Publish();
        ArmPcm();
    }
    bool FrameAt(uint64_t cycle, uint64_t& frame) const override {
        if (!linked_) return false;
        frame = frame_base_ + frames_.TicksSince(cycle);
        return true;
    }
    void OnCodecStreamChange() override { RefreshLink(); Publish(); ArmPcm(); }
    void CodecWake(uint64_t, bool) override { emu_.Get<Fatal>().Die("Au1300 AC97 codec wake signaling"); }
    void CodecGpioInterrupt(uint64_t, bool) override { emu_.Get<Fatal>().Die("Au1300 AC97 codec GPIO slot interrupt"); }
    void SaveState(StateWriter& w) override {
        AdvancePcm();
        w.Write("registers", registers_);
        w.Write("pending", pending_);
        w.Write("linked", linked_);
        w.Write("tx_active", tx_active_);
        w.Write("tx_stopping", tx_stopping_);
        w.Write("pcm_frame", pcm_frame_);
        w.Write("dac_phase", dac_phase_);
        w.Write("tx_fifo_size", static_cast<uint32_t>(tx_fifo_.size()));
        for (auto sample : tx_fifo_) w.Write("tx_fifo", sample);
        const auto frame = FrameNow();
        w.Write("frame", frame);
        frames_.TickPhaseAt(clock_->Cycles()).Save(w, "frame_phase");
        codec_->SaveState(w);
    }
    void RestoreState(StateReader& r) override {
        r.Read("registers", registers_);
        r.Read("pending", pending_);
        r.Read("linked", linked_);
        r.Read("tx_active", tx_active_);
        r.Read("tx_stopping", tx_stopping_);
        r.Read("pcm_frame", pcm_frame_);
        r.Read("dac_phase", dac_phase_);
        uint32_t size = 0u;
        r.Read("tx_fifo_size", size);
        if (size > 16u || dac_phase_ >= 48000u) r.Reject("Au1300 AC97 PCM state");
        tx_fifo_.clear();
        for (uint32_t i = 0u; i < size; ++i) { uint32_t sample = 0u; r.Read("tx_fifo", sample); tx_fifo_.push_back(sample); }
        r.Read("frame", frame_base_);
        restored_phase_.Restore(r, "frame_phase");
        codec_->RestoreState(r);
        if ((registers_[0u] != 0u && registers_[0u] != 0x24u) || registers_[1u] > 3u ||
            registers_[1u] == 1u || (registers_[9u] & ~3u) != 0u || (registers_[3u] & ~kEvents) != 0u) {
            r.Reject("Au1300 AC97 register state");
        }
        frames_.SetRate(clock_->ClockRate(), {48000u, 1u});
        frames_.AnchorAt(clock_->Cycles(), 0u, restored_phase_);
    }
    void PostRestore() override {
        output_.StopAudioOut();
        output_active_ = false;
        samples_.clear();
        codec_->PostRestore();
        Publish();
        ArmPcm();
    }

private:
    void Reset() {
        registers_ = {};
        pending_ = false;
        tx_fifo_.clear();
        tx_active_ = tx_stopping_ = false;
        dac_phase_ = 0u;
        samples_.clear();
        if (clock_) pcm_frame_ = FrameNow();
    }
    uint64_t FrameNow() const { return frame_base_ + frames_.TicksSince(clock_->Cycles()); }
    void RefreshLink() {
        const bool on = registers_[0u] == 0x24u && registers_[1u] == 3u &&
            (registers_[2u] & (1u << 26u)) != 0u && (registers_[9u] & 2u) == 0u &&
            !codec_->LinkPoweredDown();
        if (on == linked_) return;
        linked_ = on;
        pcm_frame_ = FrameNow();
        if (on) codec_->LinkStarted(FrameNow());
        else codec_->LinkStopped(FrameNow());
    }
    void ExecuteCommand() {
        const auto command = registers_[8u];
        if (((command >> 23u) & 3u) != 0u) return;
        pending_ = false;
        const auto index = (command >> 16u) & 0x7Fu;
        if ((command & (1u << 25u)) != 0u) {
            registers_[8u] = (command & ~0xFFFFu) | codec_->ReadReg(index, FrameNow());
        } else {
            codec_->WriteReg(index, static_cast<uint16_t>(command), FrameNow());
        }
        registers_[6u] |= 0x01000000u;
        RefreshLink();
    }
    uint32_t Status() const {
        const bool ready = registers_[0u] == 0x24u && registers_[1u] == 3u;
        if (!ready) return 0u;
        if ((registers_[2u] & (1u << 26u)) == 0u) return 1u;
        uint32_t status = 0x1003u;
        if (tx_fifo_.empty()) status |= 0x200u;
        if (tx_fifo_.size() == 16u) status |= 0x400u;
        if (TxRequest()) status |= 0x100u;
        if (tx_active_) status |= 0x10u;
        if (linked_) status |= 0x05000000u;
        if (pending_) status |= 0x02000000u;
        if ((registers_[6u] & ~registers_[3u]) != 0u) status |= 4u;
        return status;
    }
    void Publish() {
        if ((registers_[2u] & 0x8000000u) != 0u && TxRequest()) registers_[6u] |= 0x400u;
        emu_.Get<Au1300ModelHost>().Gpic().SetLevel(52u, (Status() & 4u) != 0u);
        emu_.Get<Au1300ModelHost>().Dma().SetRequest(16u,
            linked_ && (registers_[2u] & 0x8000000u) == 0u && TxRequest());
    }
    bool TxRequest() const {
        return 16u - tx_fifo_.size() >= (1u << ((registers_[2u] >> 28u) & 3u));
    }
    void ArmPcm() {
        if (!linked_ || !tx_active_) { clock_->Disarm(pcm_event_); return; }
        clock_->Arm(pcm_event_, frames_.CycleOfTick(pcm_frame_ + 1u - frame_base_));
    }
    void StopTx() {
        tx_active_ = tx_stopping_ = false;
        registers_[6u] |= 0x10u;
        FlushSamples();
        if (output_active_) output_.FinishAudioOut();
        output_active_ = false;
    }
    int16_t Sample(uint32_t value) const {
        const auto bits = (((registers_[2u] >> 21u) & 15u) * 2u) + 2u;
        const auto sample = static_cast<int32_t>(value << (32u - bits)) >> (32u - bits);
        return static_cast<int16_t>(bits >= 16u ? sample >> (bits - 16u) : sample * (1u << (16u - bits)));
    }
    void FlushSamples() {
        if (samples_.empty()) return;
        output_.QueueOutput(samples_.data(), static_cast<uint32_t>(samples_.size() * sizeof(int16_t)));
        samples_.clear();
    }
    void AdvancePcm() {
        if (!clock_) return;
        const auto now = FrameNow();
        if (!linked_ || !tx_active_) { pcm_frame_ = now; return; }
        if (!output_active_) {
            output_.SetFormat(codec_->PcmOutEveryFrame() ? 48000u : codec_->DacRateHz(), 2u, 16u);
            output_.BeginAudioOut();
            output_active_ = true;
        }
        const auto master = codec_->ReadReg(2u, now);
        const auto mixer = codec_->ReadReg(0x18u, now);
        const bool muted = ((master | mixer) & 0x8000u) != 0u || !codec_->DacPowered();
        double gain[2]{};
        for (uint32_t channel = 0u; channel < 2u; ++channel) {
            const auto shift = channel == 0u ? 8u : 0u;
            gain[channel] = std::pow(10.0, (12.0 - 1.5 * (((master >> shift) & 31u) + ((mixer >> shift) & 31u))) / 20.0);
        }
        while (pcm_frame_ < now && tx_active_) {
            ++pcm_frame_;
            dac_phase_ += codec_->PcmOutEveryFrame() ? 48000u : codec_->DacRateHz();
            if (dac_phase_ < 48000u) continue;
            dac_phase_ -= 48000u;
            for (uint32_t channel = 0u; channel < 2u; ++channel) {
                int16_t value = 0;
                if (tx_fifo_.empty()) registers_[6u] |= 0x100u;
                else { value = Sample(tx_fifo_.front()); tx_fifo_.pop_front(); }
                const auto scaled = muted ? 0.0 : static_cast<double>(value) * gain[channel];
                samples_.push_back(static_cast<int16_t>(std::clamp(scaled, -32768.0, 32767.0)));
            }
            if (samples_.size() >= 1024u) { FlushSamples(); emu_.Get<AudioActivityWidget>().MarkTx(); }
            if (tx_stopping_ && tx_fifo_.empty()) StopTx();
        }
    }
    std::array<uint32_t, 12u> registers_{};
    GuestCycleClock* clock_ = nullptr;
    Ac97Codec* codec_ = nullptr;
    CycleAnchoredCounter frames_;
    uint64_t frame_base_ = 0u;
    CounterPhase restored_phase_;
    bool linked_ = false;
    bool pending_ = false;
    GuestCycleClock::Event* pcm_event_ = nullptr;
    std::deque<uint32_t> tx_fifo_;
    uint64_t pcm_frame_ = 0u;
    uint32_t dac_phase_ = 0u;
    bool tx_active_ = false;
    bool tx_stopping_ = false;
    PacedWaveOut output_;
    std::vector<int16_t> samples_;
    bool output_active_ = false;
};

}
REGISTER_SERVICE(Au1300Ac97Peripheral);
}
