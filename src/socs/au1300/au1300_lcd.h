#pragma once

#include "../raster_scan_peripheral.h"

#include <array>

namespace cerfcore {

class Au1300Lcd : public RasterScanPeripheral {
public:
    using Registers = std::array<uint32_t, 0x900u / 4u>;
    explicit Au1300Lcd(CerfEmulator& emu);
    bool ShouldRegister() override;
    void OnReady() override;
    void OnShutdown() override;
    uint32_t MmioBase() const override { return 0x15000000u; }
    uint32_t MmioSize() const override { return 0x900u; }
    uint8_t ReadByte(uint32_t address) override;
    uint16_t ReadHalf(uint32_t address) override;
    uint32_t ReadWord(uint32_t address) override;
    void WriteByte(uint32_t address, uint8_t value) override;
    void WriteHalf(uint32_t address, uint16_t value) override;
    void WriteWord(uint32_t address, uint32_t value) override;
    Registers Snapshot();
    void SaveState(StateWriter& w) override;
    void RestoreState(StateReader& r) override;
    void PostRestore() override;

private:
    ScanShape ScanShapeLocked() const override;
    void FrameEdgeLocked(uint32_t edge) override;
    void ScanEdgesRan() override;
    bool EdgeRaisesInterruptLocked(uint32_t, bool) const override;
    void Reset();
    void WriteRegister(uint32_t offset, uint32_t value, uint32_t mask);
    bool Known(uint32_t offset) const;
    void ClockChanged();
    GuestCycleClock::Rate PixelClock() const;
    Registers registers_{};
    bool shutdown_ = false;
};

}
