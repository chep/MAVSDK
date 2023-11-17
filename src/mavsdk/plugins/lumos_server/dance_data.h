#pragma once

#include <vector>

#include <mavlink/lightshow/mavlink.h>

namespace mavsdk {

class DanceData {
public:
    DanceData() {}
    ~DanceData() {}

    mavlink_dance_fram_ftp_t parse(mavlink_dance_fram_ftp_t& req);
    bool is_valid() const { return _is_valid; }
    size_t size() const { return _data.size(); }
    const uint8_t* data() const { return _data.data(); }

private:
    static constexpr uint8_t ACK = 0;
    static constexpr uint8_t NACK = 1;

    bool compare_crc(uint32_t crc);

    std::vector<uint8_t> _data;
    bool _is_valid{false};
};

} // namespace mavsdk
