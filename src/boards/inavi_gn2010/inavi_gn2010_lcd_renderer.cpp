#include "inavi_gn2010_family_id.h"
#include "../../socs/au1300/au1300_lcd.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/byte_order.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/cpu/emulated_memory.h>
#include <cerfcore/host/panel_frame_renderer.h>

#include <algorithm>
#include <array>
#include <vector>

namespace cerfcore {
namespace {

class InaviGn2010LcdRenderer : public PanelFrameRenderer {
public:
    using PanelFrameRenderer::PanelFrameRenderer;
    bool ShouldRegister() override {
        return emu_.Get<BoardContext>().GetBoardFamilyId() == BoardFamilyId::InaviGn2010;
    }
    void PresentedSize(uint32_t& width, uint32_t& height) override {
        const auto registers = emu_.Get<Au1300Lcd>().Snapshot();
        width = ((registers[1u] >> 19u) & 0x7FFu) + 1u;
        height = ((registers[1u] >> 8u) & 0x7FFu) + 1u;
    }
    bool HasFrame() override {
        const auto registers = emu_.Get<Au1300Lcd>().Snapshot();
        if ((registers[1u] & 0x80000000u) == 0u) return false;
        if (latch_.Latched()) return true;
        for (uint32_t i = 0u; i < 4u; ++i) {
            if ((registers[9u] & (1u << i)) == 0u) continue;
            const auto index = (0x100u + i * 0x20u) / 4u;
            const auto buffer = registers[index + 3u + ((registers[index + 5u] >> 1u) & 1u)];
            const auto stride = (registers[index + 2u] >> 8u) & 0x1FFFu;
            const auto height = (registers[index + 1u] & 0x7FFu) + 1u;
            if (stride != 0u && latch_.ProbeAndLatch(emu_.Get<EmulatedMemory>(), buffer,
                                                    static_cast<size_t>(stride) * height)) return true;
        }
        return false;
    }
    void RenderInto(uint32_t* output, uint32_t width, uint32_t height) override {
        const auto registers = emu_.Get<Au1300Lcd>().Snapshot();
        memory_      = &emu_.Get<EmulatedMemory>();
        cached_page_ = kNoPage;
        std::fill_n(output, static_cast<size_t>(width) * height, 0xFF000000u);
        if ((registers[1u] & 0x80000000u) == 0u) return;
        const auto columns = std::min(width, ((registers[1u] >> 19u) & 0x7FFu) + 1u);
        const auto rows = std::min(height, ((registers[1u] >> 8u) & 0x7FFu) + 1u);
        for (uint32_t y = 0u; y < rows; ++y) {
            uint32_t* const line = output + static_cast<size_t>(y) * width;
            if (!RenderDirectRow(registers, line, columns, y)) RenderComposedRow(registers, line, columns, y);
        }
    }

private:
    static constexpr uint32_t kPageBytes = 4096u;
    static constexpr uint32_t kNoPage    = 1u;

    const uint8_t* FramebufferWord(uint32_t address) {
        const uint32_t page = address & ~(kPageBytes - 1u);
        if (address - page > kPageBytes - 4u) return memory_->TryTranslateRange(address, 4u);
        if (page != cached_page_) {
            cached_page_ = page;
            cached_host_ = memory_->TryTranslateRange(page, kPageBytes);
        }
        return cached_host_ ? cached_host_ + (address - page) : memory_->TryTranslateRange(address, 4u);
    }

    void RenderComposedRow(const Au1300Lcd::Registers& registers, uint32_t* line, uint32_t columns, uint32_t y) {
        const auto background = 0xFF000000u | registers[2u];
        for (uint32_t x = 0u; x < columns; ++x) {
            std::array<Pixel, 2u> pipe{};
            for (uint32_t i = 0u; i < 4u; ++i) {
                if ((registers[9u] & (1u << i)) == 0u) continue;
                const auto index = (0x100u + i * 0x20u) / 4u;
                const auto control = registers[index + 1u];
                const auto side = (control >> 29u) & 1u;
                const auto priority = control >> 30u;
                if (pipe[side].present && priority < pipe[side].priority) continue;
                Pixel pixel;
                if (!WindowPixel(registers, index, x, y, pixel.color)) continue;
                pixel.present = true;
                pixel.priority = priority;
                pipe[side] = pixel;
            }
            auto color = background;
            if (pipe[0u].present || pipe[1u].present) {
                const bool right = pipe[1u].present &&
                    (!pipe[0u].present || pipe[1u].priority > pipe[0u].priority);
                const auto top = right ? 1u : 0u;
                const auto bottom = top ^ 1u;
                color = pipe[top].color;
                if (pipe[bottom].present) color = Blend(color, pipe[bottom].color);
            }
            color = Cursor(registers, x, y, color);
            if ((registers[1u] & 0x40u) != 0u) {
                color = (registers[1u] & 0x80u) != 0u ? 0xFFFFFFFFu : 0xFF000000u;
            }
            line[x] = 0xFF000000u | (color & registers[0x50u / 4u]);
        }
    }

    bool CursorOnRow(const Au1300Lcd::Registers& registers, uint32_t y) const {
        if ((registers[0x30u / 4u] & 1u) == 0u) return false;
        const auto position = registers[0x34u / 4u];
        const auto origin_y = position & 0x7FFu;
        return y >= origin_y && y - origin_y + ((position >> 11u) & 31u) < 32u;
    }

    bool RenderDirectRow(const Au1300Lcd::Registers& registers, uint32_t* line, uint32_t columns, uint32_t y) {
        if ((registers[1u] & 0x40u) != 0u || CursorOnRow(registers, y)) return false;
        uint32_t found = 0u;
        uint32_t index = 0u;
        for (uint32_t i = 0u; i < 4u; ++i) {
            if ((registers[9u] & (1u << i)) == 0u) continue;
            const auto candidate = (0x100u + i * 0x20u) / 4u;
            const auto top = (registers[candidate] >> 10u) & 0x7FFu;
            if (y < top || y - top > (registers[candidate + 1u] & 0x7FFu)) continue;
            if ((registers[candidate] >> 21u) >= columns) continue;
            index = candidate;
            ++found;
        }
        const auto mask = registers[0x50u / 4u];
        const auto background = 0xFF000000u | (registers[2u] & mask);
        if (found == 0u) {
            std::fill_n(line, columns, background);
            return true;
        }
        if (found != 1u) return false;
        const auto origin = registers[index];
        const auto control = registers[index + 1u];
        const auto config = registers[index + 2u];
        const auto format = (control >> 25u) & 15u;
        const bool wide = format == 12u || format == 13u;
        const bool half = format == 4u || format == 5u || format == 6u || format == 7u || format == 10u || format == 11u;
        if ((!wide && !half) || ((config >> 21u) & 3u) != 0u || (config & 0x3000000u) != 0u) return false;
        const auto left = origin >> 21u;
        const auto count = std::min(((control >> 11u) & 0x7FFu) + 1u, columns - left);
        const auto buffer = registers[index + 3u + ((registers[index + 5u] >> 1u) & 1u)];
        const uint64_t start = static_cast<uint64_t>(buffer) +
                               static_cast<uint64_t>(y - ((origin >> 10u) & 0x7FFu)) * ((config >> 8u) & 0x1FFFu);
        const uint64_t bytes = wide ? uint64_t{count} * 4u : ((uint64_t{count} + 1u) / 2u) * 4u;
        if (start + bytes - 1u > UINT32_MAX) return false;
        const uint8_t* const source = memory_->TryTranslateRange(start, bytes);
        if (!source) return false;
        const bool swap = (control & 0x1000000u) != 0u;
        std::fill_n(line, left, background);
        uint32_t* const target = line + left;
        if (wide) {
            for (uint32_t x = 0u; x < count; ++x) {
                const auto color = Decode(cerf::le::U32(source + static_cast<size_t>(x) * 4u), format);
                target[x] = 0xFF000000u | ((swap ? SwapRedBlue(color) : color) & mask);
            }
        } else {
            const uint32_t* const table = DirectTable(format, swap, mask);
            const auto first = ((control >> 22u) & 1u) ^ 1u;
            for (uint32_t x = 0u; x < count; ++x) {
                const auto word = cerf::le::U32(source + static_cast<size_t>(x / 2u) * 4u);
                target[x] = table[(word >> (((x & 1u) ^ first) * 16u)) & 0xFFFFu];
            }
        }
        std::fill_n(target + count, columns - left - count, background);
        return true;
    }

    const uint32_t* DirectTable(uint32_t format, bool swap, uint32_t mask) {
        const uint64_t key = (uint64_t{format} << 33u) | (uint64_t{swap} << 32u) | mask;
        if (direct_table_.empty() || key != direct_key_) {
            direct_table_.resize(0x10000u);
            for (uint32_t value = 0u; value < 0x10000u; ++value) {
                const auto color = Decode(value, format);
                direct_table_[value] = 0xFF000000u | ((swap ? SwapRedBlue(color) : color) & mask);
            }
            direct_key_ = key;
        }
        return direct_table_.data();
    }

    static uint32_t SwapRedBlue(uint32_t color) {
        return (color & 0xFF00FF00u) | ((color & 0xFFu) << 16u) | ((color >> 16u) & 0xFFu);
    }

    EmulatedMemory*       memory_      = nullptr;
    uint32_t              cached_page_ = kNoPage;
    const uint8_t*        cached_host_ = nullptr;
    std::vector<uint32_t> direct_table_;
    uint64_t              direct_key_  = 0u;

    struct Pixel {
        uint32_t color = 0u;
        uint32_t priority = 0u;
        bool present = false;
    };
    uint32_t Blend(uint32_t top, uint32_t bottom) const {
        const auto alpha = top >> 24u;
        uint32_t result = 0xFF000000u;
        for (uint32_t shift = 0u; shift < 24u; shift += 8u) {
            const auto a = (top >> shift) & 0xFFu;
            const auto b = (bottom >> shift) & 0xFFu;
            result |= ((a * alpha + b * (255u - alpha) + 127u) / 255u) << shift;
        }
        return result;
    }
    uint32_t Channel(uint32_t value, uint32_t width) const {
        const auto mask = (1u << width) - 1u;
        return (value & mask) * 255u / mask;
    }
    uint32_t Decode(uint32_t value, uint32_t format) const {
        uint32_t red = 0u, green = 0u, blue = 0u, alpha = 255u;
        switch (format) {
        case 0u: case 1u: case 2u: case 3u:
            red = green = blue = Channel(value, 1u << format);
            break;
        case 4u:
            red = Channel(value >> 8u, 4u); green = Channel(value >> 4u, 4u); blue = Channel(value, 4u);
            break;
        case 5u:
            red = Channel(value >> 10u, 6u); green = Channel(value >> 5u, 5u); blue = Channel(value, 5u);
            break;
        case 6u:
            red = Channel(value >> 11u, 5u); green = Channel(value >> 5u, 6u); blue = Channel(value, 5u);
            break;
        case 7u:
            red = Channel(value >> 11u, 5u); green = Channel(value >> 6u, 5u); blue = Channel(value, 6u);
            break;
        case 10u:
            alpha = (value & 0x8000u) != 0u ? 255u : 0u;
            red = Channel(value >> 10u, 5u); green = Channel(value >> 5u, 5u); blue = Channel(value, 5u);
            break;
        case 11u:
            alpha = (value & 1u) != 0u ? 255u : 0u;
            red = Channel(value >> 11u, 5u); green = Channel(value >> 6u, 5u); blue = Channel(value >> 1u, 5u);
            break;
        case 12u: return value | 0xFF000000u;
        case 13u: return value;
        default: emu_.Get<Fatal>().Die("Au1300 LCD pixel format=%u", format);
        }
        return (alpha << 24u) | (red << 16u) | (green << 8u) | blue;
    }
    bool WindowPixel(const Au1300Lcd::Registers& registers, uint32_t index,
                     uint32_t x, uint32_t y, uint32_t& color) {
        const auto origin = registers[index];
        const auto horizontal = origin >> 21u;
        const auto vertical = (origin >> 10u) & 0x7FFu;
        if (x < horizontal || y < vertical) return false;
        x -= horizontal;
        y -= vertical;
        const auto control = registers[index + 1u];
        const auto width = ((control >> 11u) & 0x7FFu) + 1u;
        const auto height = (control & 0x7FFu) + 1u;
        if (x >= width || y >= height) return false;
        const auto format = (control >> 25u) & 15u;
        const auto bits = format < 4u ? 1u << format : format < 12u ? 16u : 32u;
        const auto config = registers[index + 2u];
        const auto mode = (config >> 21u) & 3u;
        if ((config & 0x3000000u) != 0u) emu_.Get<Fatal>().Die("Au1300 LCD color key mode=%u", config >> 24u);
        const auto buffer = registers[index + 3u + ((registers[index + 5u] >> 1u) & 1u)];
        uint32_t value;
        if (mode == 3u) {
            const auto scale_x = (config >> 4u) & 15u;
            const auto scale_y = config & 15u;
            if (format > 1u || scale_x > 2u || scale_y > 2u) {
                emu_.Get<Fatal>().Die("Au1300 LCD on-chip format=%u scale=%u/%u", format, scale_x, scale_y);
            }
            const auto position = (buffer * 8u) + ((y >> scale_y) * ((config >> 8u) & 0x1FFFu) +
                                                   (x >> scale_x)) * bits;
            if (position >= 8192u) return false;
            value = (registers[0x100u + position / 32u] >> (32u - bits - position % 32u)) & ((1u << bits) - 1u);
            color = registers[0x38u / 4u + value];
        } else {
            const auto stride = (config >> 8u) & 0x1FFFu;
            const uint64_t address = static_cast<uint64_t>(buffer) + static_cast<uint64_t>(y) * stride +
                                     (static_cast<uint64_t>(x) * bits / 32u) * 4u;
            if (address > UINT32_MAX) return false;
            const auto* source = FramebufferWord(static_cast<uint32_t>(address));
            if (!source) return false;
            value = cerf::le::U32(source);
            const auto order = (control >> 22u) & 3u;
            if (bits <= 8u) {
                const auto per_byte = 8u / bits;
                const auto position = x % (32u / bits);
                auto byte = 3u - position / per_byte;
                auto group = per_byte - 1u - position % per_byte;
                if ((order & 1u) != 0u) byte ^= 3u;
                if ((order & 2u) != 0u) group ^= per_byte - 1u;
                value = (value >> (byte * 8u + group * bits)) & ((1u << bits) - 1u);
            } else if (bits == 16u) {
                value = (value >> ((((x & 1u) ^ (order & 1u)) ^ 1u) * 16u)) & 0xFFFFu;
            }
            color = mode == 1u ? registers[0x100u + (value & 0xFFu)] : Decode(value, format);
            if (mode == 2u) {
                uint32_t corrected = 0u;
                for (uint32_t shift = 0u; shift < 32u; shift += 8u) {
                    corrected |= registers[0x100u + ((color >> shift) & 0xFFu)] & (0xFFu << shift);
                }
                color = corrected;
            }
        }
        if ((control & 0x1000000u) != 0u) color = SwapRedBlue(color);
        if ((origin & 2u) != 0u) color = (color & 0xFFFFFFu) | (((origin >> 2u) & 0xFFu) << 24u);
        return true;
    }
    uint32_t Cursor(const Au1300Lcd::Registers& registers, uint32_t x, uint32_t y, uint32_t color) const {
        if ((registers[0x30u / 4u] & 1u) == 0u) return color;
        const auto position = registers[0x34u / 4u];
        const auto origin_x = (position >> 16u) & 0x7FFu;
        const auto origin_y = position & 0x7FFu;
        if (x < origin_x || y < origin_y) return color;
        x = x - origin_x + (position >> 27u);
        y = y - origin_y + ((position >> 11u) & 31u);
        if (x >= 32u || y >= 32u) return color;
        const auto pixel = x + y * 32u;
        const auto pattern = registers[0x800u / 4u + pixel / 16u];
        const auto cursor = registers[0x38u / 4u + ((pattern >> ((pixel % 16u) * 2u)) & 3u)];
        return Blend(cursor, color);
    }
};

}
REGISTER_SERVICE_AS(InaviGn2010LcdRenderer, PanelFrameRenderer);
}
