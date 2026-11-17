#pragma once

#include <cerfcore/core/service.h>

#include <array>
#include <mutex>

namespace cerfcore {

class HostMotionFeed : public Service {
public:
    using Service::Service;

    using Vector = std::array<double, 3>;

    void SetAccelerationG(const Vector& g);
    void SetMagneticFieldMicroTesla(const Vector& field);

    Vector AccelerationG() const;
    Vector MagneticFieldMicroTesla() const;

private:
    mutable std::mutex mu_;
    Vector             acceleration_g_{0.0, 0.0, 1.0};
    Vector             field_micro_tesla_{30.0, 0.0, -35.0};
};

}
