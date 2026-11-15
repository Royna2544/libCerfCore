#pragma once

#include <cstdint>

#include <cerfcore/core/service.h>
#include <cerfengine/mips/mips_system_host.h>
#include <cerfengine/mips/mips_tlb_context.h>

namespace cerfengine { struct MipsCpuState; }

namespace cerfcore {

class StateWriter;
class StateReader;

class MipsTlbModel : public Service {
public:
    using Service::Service;

    virtual cerfengine::MipsTlbHooks Hooks()                     = 0;
    virtual uint32_t RandomIndex(cerfengine::MipsCpuState* st) = 0;

    void SaveState(StateWriter& w) const;
    void RestoreState(StateReader& r);
    void ResetRandomAt(uint64_t cycle) { random_anchor_ = cycle; }

protected:
    uint32_t NextRandom(uint32_t first, uint32_t nb);
    uint32_t CountdownRandom(uint32_t first, uint32_t count, uint64_t cycle);

    static void FlushAll(cerfengine::MipsTlbContext& ctx);

private:
    uint32_t lcg_seed_        = 1;
    uint32_t prev_random_idx_ = 0;
    uint64_t random_anchor_   = 0;
};

}
