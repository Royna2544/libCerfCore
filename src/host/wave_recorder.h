#pragma once

#include <cerfcore/platform/host_file.h>

#include <cstdint>
#include <string>

namespace cerfcore {

class WaveRecorder {
public:
    WaveRecorder() = default;
    ~WaveRecorder();
    WaveRecorder(const WaveRecorder&)            = delete;
    WaveRecorder& operator=(const WaveRecorder&) = delete;

    bool     Open(const std::string& path, uint32_t rate_hz, uint16_t channels, uint16_t bits);
    void     Append(const void* bytes, uint32_t length);
    void     AppendSilence(uint64_t frames);
    void     Close();
    bool     IsOpen() const { return file_.IsOpen(); }
    uint32_t RateHz() const { return rate_hz_; }

private:
    void WriteHeader(uint32_t data_bytes);

    host_platform::HostFile file_;
    uint32_t                rate_hz_    = 0;
    uint16_t                channels_   = 0;
    uint16_t                bits_       = 0;
    uint32_t                data_bytes_ = 0;
};

}
