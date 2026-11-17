#include <cerfcore/host/host_nmea_feed.h>

#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/log.h>

namespace cerfcore {

REGISTER_SERVICE(HostNmeaFeed);

void HostNmeaFeed::PushSentence(std::string_view sentence) {
    while (!sentence.empty() && (sentence.back() == '\r' || sentence.back() == '\n')) {
        sentence.remove_suffix(1u);
    }
    if (sentence.size() < 6u || sentence.front() != '$') {
        LOG(Caution, "[NMEA] dropped a host line that is not an NMEA sentence (%zu bytes)\n",
            sentence.size());
        return;
    }
    std::string line(sentence);
    line += "\r\n";
    std::lock_guard<std::mutex> lk(mu_);
    if (receiver_) receiver_(line);
}

void HostNmeaFeed::SetReceiver(SentenceFn receiver) {
    std::lock_guard<std::mutex> lk(mu_);
    receiver_ = std::move(receiver);
}

}
