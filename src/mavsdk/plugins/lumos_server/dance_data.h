#pragma once

#include <vector>

#include <mavlink/lightshow/mavlink.h>

namespace mavsdk {

constexpr int MAX_POLY = 20;
constexpr int DANCE_PARAMS_SIZE = 20 + MAX_POLY * 8;

struct DanceParams {
    struct PolygonVertex {
        float x; // North coordinate
        float y; // East coordinate
    };

    float lon;
    float lat;
    float alt;

    int32_t gps_start; // Time to start
    float gf_alt;
    struct PolygonVertex poly[MAX_POLY];
};

class DanceData {
public:
    DanceData() : _data(0) {}
    ~DanceData() {}

    mavlink_dance_fram_ftp_t parse(mavlink_dance_fram_ftp_t& req);
    bool is_valid() const { return _is_valid; }
    size_t size() const { return _data.size(); }
    const uint8_t* data() const { return _data.data(); }
    DanceParams params() const { return _params; }
    void clear()
    {
        _data.clear();
        _is_valid = false;
    }

    static constexpr uint8_t ACK = 0;
    static constexpr uint8_t NACK = 1;

private:
    bool compare_crc(uint32_t crc);
    uint8_t set_params(const mavlink_dance_fram_ftp_t& req);

    std::vector<uint8_t> _data;
    bool _is_valid{false};

    DanceParams _params;
};

} // namespace mavsdk
