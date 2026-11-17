#include <cerfcore/host/host_motion_feed.h>

#include <cerfcore/core/cerf_emulator.h>

namespace cerfcore {

REGISTER_SERVICE(HostMotionFeed);

void HostMotionFeed::SetAccelerationG(const Vector& g) {
    std::lock_guard<std::mutex> lk(mu_);
    acceleration_g_ = g;
}

void HostMotionFeed::SetMagneticFieldMicroTesla(const Vector& field) {
    std::lock_guard<std::mutex> lk(mu_);
    field_micro_tesla_ = field;
}

HostMotionFeed::Vector HostMotionFeed::AccelerationG() const {
    std::lock_guard<std::mutex> lk(mu_);
    return acceleration_g_;
}

HostMotionFeed::Vector HostMotionFeed::MagneticFieldMicroTesla() const {
    std::lock_guard<std::mutex> lk(mu_);
    return field_micro_tesla_;
}

}
