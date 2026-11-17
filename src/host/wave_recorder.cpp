#include "wave_recorder.h"

#include <cerfcore/core/byte_order.h>

#include <algorithm>
#include <array>

namespace cerfcore {

namespace {

constexpr uint32_t kHeaderBytes = 44u;
constexpr uint32_t kMaxDataBytes = 0xFFFFFFFFu - kHeaderBytes;

}

WaveRecorder::~WaveRecorder() { Close(); }

bool WaveRecorder::Open(const std::string& path, uint32_t rate_hz, uint16_t channels, uint16_t bits) {
    Close();
    if (rate_hz == 0u || channels == 0u || bits == 0u || bits % 8u != 0u) return false;
    if (!file_.Open(path, host_platform::FileMode::kWriteNew)) return false;
    rate_hz_    = rate_hz;
    channels_   = channels;
    bits_       = bits;
    data_bytes_ = 0u;
    WriteHeader(0u);
    return true;
}

void WaveRecorder::WriteHeader(uint32_t data_bytes) {
    std::array<uint8_t, kHeaderBytes> h{};
    const uint16_t block = static_cast<uint16_t>(channels_ * (bits_ / 8u));
    std::copy_n("RIFF", 4, h.data());
    cerf::le::Put32(h.data() + 4, kHeaderBytes - 8u + data_bytes);
    std::copy_n("WAVEfmt ", 8, h.data() + 8);
    cerf::le::Put32(h.data() + 16, 16u);
    cerf::le::Put16(h.data() + 20, 1u);
    cerf::le::Put16(h.data() + 22, channels_);
    cerf::le::Put32(h.data() + 24, rate_hz_);
    cerf::le::Put32(h.data() + 28, rate_hz_ * block);
    cerf::le::Put16(h.data() + 32, block);
    cerf::le::Put16(h.data() + 34, bits_);
    std::copy_n("data", 4, h.data() + 36);
    cerf::le::Put32(h.data() + 40, data_bytes);
    uint32_t done = 0u;
    if (file_.SeekTo(0u)) file_.Write(h.data(), kHeaderBytes, done);
    file_.SeekToEnd();
}

void WaveRecorder::Append(const void* bytes, uint32_t length) {
    if (!file_.IsOpen() || length == 0u) return;
    length = std::min(length, kMaxDataBytes - data_bytes_);
    uint32_t done = 0u;
    file_.Write(bytes, length, done);
    data_bytes_ += done;
}

void WaveRecorder::AppendSilence(uint64_t frames) {
    static constexpr uint8_t kZeros[4096] = {};
    const uint64_t frame_bytes = static_cast<uint64_t>(channels_) * (bits_ / 8u);
    uint64_t       bytes       = frames * frame_bytes;
    while (file_.IsOpen() && bytes != 0u && data_bytes_ < kMaxDataBytes) {
        const uint32_t n = static_cast<uint32_t>(std::min<uint64_t>(bytes, sizeof(kZeros) - sizeof(kZeros) % frame_bytes));
        Append(kZeros, n);
        bytes -= n;
    }
}

void WaveRecorder::Close() {
    if (!file_.IsOpen()) return;
    WriteHeader(data_bytes_);
    file_.Close();
}

}
