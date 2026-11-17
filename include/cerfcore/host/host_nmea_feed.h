#pragma once

#include <cerfcore/core/service.h>

#include <functional>
#include <mutex>
#include <string>
#include <string_view>

namespace cerfcore {

class HostNmeaFeed : public Service {
public:
    using Service::Service;

    using SentenceFn = std::function<void(const std::string&)>;

    void PushSentence(std::string_view sentence);
    void SetReceiver(SentenceFn receiver);

private:
    std::mutex mu_;
    SentenceFn receiver_;
};

}
