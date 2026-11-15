#include "au1300_lcd.h"

#include "au1300_id.h"
#include "au1300_clock_model.h"
#include "au1300_gpic_model.h"
#include "au1300_model_host.h"
#include "../../peripherals/peripheral_dispatcher.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/host/host_presentation.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>

namespace cerfcore {

REGISTER_SERVICE(Au1300Lcd);

Au1300Lcd::Au1300Lcd(CerfEmulator& emu) : RasterScanPeripheral(emu) { Reset(); }

bool Au1300Lcd::ShouldRegister() {
    return emu_.Get<BoardContext>().GetSocId() == SocId::Au1300;
}

void Au1300Lcd::Reset() {
    registers_ = {};
    registers_[0x50u / 4u] = 0xFFFFFFu;
    registers_[0x54u / 4u] = 0x2F2F2F2Fu;
    shutdown_ = false;
}

void Au1300Lcd::OnReady() {
    AttachScanClock();
    emu_.Get<Au1300ModelHost>().Sys().RegisterDerivedClockListener([this] { ClockChanged(); });
    emu_.Get<PeripheralDispatcher>().Register(this);
    auto& reset = emu_.Get<GuestCpuReset>();
    reset.RegisterResetListener([this](ResetLineKind) {
        std::lock_guard<std::mutex> lock(state_mtx_);
        StopScanLocked();
        Reset();
    });
    reset.RegisterResetReleaseListener([this] { ScanEdgesRan(); });
}

void Au1300Lcd::OnShutdown() {
    std::lock_guard<std::mutex> lock(state_mtx_);
    StopScanLocked();
}

bool Au1300Lcd::Known(uint32_t offset) const {
    if (offset >= 0x400u && offset < 0x900u) return true;
    if (offset >= 0x100u && offset < 0x180u) return (offset & 0x1Fu) <= 0x14u;
    return offset >= 4u && offset <= 0x54u && offset != 0x20u;
}

uint8_t Au1300Lcd::ReadByte(uint32_t address) {
    return static_cast<uint8_t>(ReadWord(address & ~3u) >> ((address & 3u) * 8u));
}

uint16_t Au1300Lcd::ReadHalf(uint32_t address) {
    if ((address & 1u) != 0u) HaltUnsupportedAccess("LCD ReadHalf", address, 0u);
    return static_cast<uint16_t>(ReadWord(address & ~3u) >> ((address & 2u) * 8u));
}

uint32_t Au1300Lcd::ReadWord(uint32_t address) {
    const auto offset = address - MmioBase();
    if ((offset & 3u) != 0u || !Known(offset)) HaltUnsupportedAccess("LCD ReadWord", address, 0u);
    uint32_t value;
    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        CatchUpLocked(ScanNow());
        value = registers_[offset / 4u];
    }
    ScanEdgesRan();
    return value;
}

void Au1300Lcd::WriteByte(uint32_t address, uint8_t value) {
    const auto shift = (address & 3u) * 8u;
    WriteRegister((address - MmioBase()) & ~3u, static_cast<uint32_t>(value) << shift, 0xFFu << shift);
}

void Au1300Lcd::WriteHalf(uint32_t address, uint16_t value) {
    if ((address & 1u) != 0u) HaltUnsupportedAccess("LCD WriteHalf", address, value);
    const auto shift = (address & 2u) * 8u;
    WriteRegister((address - MmioBase()) & ~3u, static_cast<uint32_t>(value) << shift, 0xFFFFu << shift);
}

void Au1300Lcd::WriteWord(uint32_t address, uint32_t value) {
    if ((address & 3u) != 0u) HaltUnsupportedAccess("LCD WriteWord", address, value);
    WriteRegister(address - MmioBase(), value, 0xFFFFFFFFu);
}

void Au1300Lcd::WriteRegister(uint32_t offset, uint32_t value, uint32_t mask) {
    if (!Known(offset)) {
        if (value == 0u) return;
        HaltUnsupportedAccess("LCD reserved write", MmioBase() + offset, value);
    }
    bool panel_enabled = false;
    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        const auto now = ScanNow();
        CatchUpLocked(now);
        auto& reg = registers_[offset / 4u];
        const auto previous = reg;
        if (offset == 0x48u) reg &= ~(value & mask);
        else {
            if (offset >= 0x100u && offset < 0x180u && (offset & 0x1Fu) == 0x14u) mask &= 1u;
            reg = (reg & ~mask) | (value & mask);
        }
        if (offset == 4u && ((previous ^ reg) & 0x80000000u) != 0u) {
            if ((reg & 0x80000000u) != 0u) {
                shutdown_ = false;
                registers_[0x48u / 4u] &= ~4u;
                StartScanLocked(now);
                if (PixelClock().num == 0u) PauseScanLocked(now);
                panel_enabled = true;
            } else {
                shutdown_ = true;
            }
        }
        RearmScanLocked();
    }
    if (panel_enabled) emu_.Get<HostPresentation>().PanelEnabled();
    ScanEdgesRan();
}

GuestCycleClock::Rate Au1300Lcd::PixelClock() const {
    auto& sys = emu_.Get<Au1300ModelHost>().Sys();
    const auto select = sys.Read(0x28u) & 0x1Fu;
    const auto mux = select >> 2u;
    uint64_t numerator = 12000000ull * (sys.Read(0x64u) & 0x3Fu);
    uint64_t denominator = 1u;
    if (mux == 0u) numerator = 0u;
    if (mux >= 2u) {
        const auto frequency = mux - 2u;
        const auto field = sys.Read(frequency < 3u ? 0x20u : 0x24u) >> ((frequency % 3u) * 10u);
        if ((field & 2u) == 0u) numerator = 0u;
        else if ((field & 1u) == 0u) numerator = sys.CpuHz();
        denominator *= 2u * (((field >> 2u) & 0xFFu) + 1u);
    }
    if ((select & 1u) != 0u) denominator *= (select & 2u) != 0u ? 2u : 4u;
    const auto lcd_clock = registers_[0x14u / 4u];
    if ((lcd_clock & 0x400000u) != 0u) {
        HaltUnsupportedAccess("LCD external pixel clock", MmioBase() + 0x14u, lcd_clock);
    }
    if ((lcd_clock & 0x80000u) == 0u) denominator *= 2u * ((lcd_clock & 0x3FFu) + 1u);
    return {numerator, denominator};
}

Au1300Lcd::ScanShape Au1300Lcd::ScanShapeLocked() const {
    const auto screen = registers_[1u];
    const uint64_t width = ((screen >> 19u) & 0x7FFu) + 1u;
    const uint64_t height = ((screen >> 8u) & 0x7FFu) + 1u;
    const auto horizontal = registers_[3u];
    const auto vertical = registers_[4u];
    const uint64_t line = width + ((horizontal >> 18u) & 0x1FFu) +
                          ((horizontal >> 9u) & 0x1FFu) + (horizontal & 0x1FFu) + 3u;
    const uint64_t retrace = ((vertical >> 9u) & 0x1FFu) + (vertical & 0x1FFu) + 2u;
    const uint64_t lines = height + retrace + ((vertical >> 18u) & 0x1FFu) + 1u;
    auto rate = PixelClock();
    if (rate.num == 0u) rate.num = 1u;
    return {rate, {line * lines, {line * retrace, line * lines}, 2u}};
}

void Au1300Lcd::FrameEdgeLocked(uint32_t edge) {
    if (edge == 1u) {
        registers_[0x48u / 4u] |= 1u;
        for (uint32_t i = 0u; i < 4u; ++i) {
            auto& control = registers_[(0x114u + i * 0x20u) / 4u];
            if ((registers_[(0x108u + i * 0x20u) / 4u] & 0x800000u) != 0u) control ^= 2u;
            else control = (control & 1u) * 3u;
        }
        if (shutdown_) {
            registers_[0x48u / 4u] |= 4u;
            shutdown_ = false;
            StopScanLocked();
        }
    } else {
        registers_[0x48u / 4u] |= 2u;
    }
}

bool Au1300Lcd::EdgeRaisesInterruptLocked(uint32_t, bool) const { return true; }

void Au1300Lcd::ScanEdgesRan() {
    bool level;
    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        level = (registers_[0x48u / 4u] & registers_[0x4Cu / 4u]) != 0u;
    }
    emu_.Get<Au1300ModelHost>().Gpic().SetLevel(91u, level);
}

void Au1300Lcd::ClockChanged() {
    bool running;
    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        running = PixelClock().num != 0u;
    }
    OnScanFunctionalClockChange(running);
}

Au1300Lcd::Registers Au1300Lcd::Snapshot() {
    std::lock_guard<std::mutex> lock(state_mtx_);
    return registers_;
}

void Au1300Lcd::SaveState(StateWriter& w) {
    std::lock_guard<std::mutex> lock(state_mtx_);
    CatchUpLocked(ScanNow());
    w.Write("registers", registers_);
    w.Write("shutdown", shutdown_);
    SaveScanLocked(w);
}

void Au1300Lcd::RestoreState(StateReader& r) {
    std::lock_guard<std::mutex> lock(state_mtx_);
    r.Read("registers", registers_);
    r.Read("shutdown", shutdown_);
    RestoreScanLocked(r);
}

void Au1300Lcd::PostRestore() {
    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        ResumeScanLocked(ScanNow());
    }
    ScanEdgesRan();
}

}
