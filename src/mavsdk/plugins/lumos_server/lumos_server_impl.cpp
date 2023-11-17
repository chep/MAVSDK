#include "lumos_server_impl.h"

namespace mavsdk {

static const std::chrono::milliseconds STATUS_MESSAGE_INTERVAL(1000);

LumosServerImpl::LumosServerImpl(std::shared_ptr<ServerComponent> server_component) :
    ServerPluginImplBase(server_component),
    _boot_time(std::chrono::steady_clock::now())
{
    _server_component_impl->register_plugin(this);
}

LumosServerImpl::~LumosServerImpl()
{
    _server_component_impl->unregister_plugin(this);
}

void LumosServerImpl::init()
{
    // drone info thread
    _status_thread = std::thread(std::bind(&LumosServerImpl::drone_status_thread, this));

    // PX4 messages handlers
    _server_component_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_BATTERY_STATUS,
        std::bind(&LumosServerImpl::battery_status_handler, this, std::placeholders::_1),
        nullptr);
    _server_component_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        std::bind(&LumosServerImpl::position_handler, this, std::placeholders::_1),
        nullptr);
    _server_component_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_GPS_STATUS,
        std::bind(&LumosServerImpl::gps_status_handler, this, std::placeholders::_1),
        nullptr);
    _server_component_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_GPS_RAW_INT,
        std::bind(&LumosServerImpl::gps_raw_handler, this, std::placeholders::_1),
        nullptr);
    _server_component_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_HIGHRES_IMU,
        std::bind(&LumosServerImpl::highres_imu_handler, this, std::placeholders::_1),
        nullptr);

    // GCS messages handlers
    _server_component_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_DANCE_FRAM_FTP,
        std::bind(&LumosServerImpl::fram_ftp_handler, this, std::placeholders::_1),
        nullptr);
}

void LumosServerImpl::deinit()
{
    _stop_threads = true;
    if (_status_thread.joinable())
        _status_thread.join();
}

void LumosServerImpl::drone_status_thread()
{
    while (!_stop_threads) {
        std::this_thread::sleep_for(STATUS_MESSAGE_INTERVAL);
        if (!_info_never_set) {
            std::lock_guard<std::mutex> lock(_status_mutex);
            auto result = _server_component_impl->queue_message(
                [&](MavlinkAddress mavlink_address, uint8_t channel) {
                    mavlink_message_t message;
                    char uuid[12];
                    std::memcpy(uuid, _drone_info.uuid.c_str(), sizeof(uuid));
                    const auto time = std::chrono::steady_clock::now();
                    mavlink_msg_drone_status_pack_chan(
                        mavlink_address.system_id,
                        mavlink_address.component_id,
                        channel,
                        &message,
                        reinterpret_cast<uint8_t*>(uuid),
                        _drone_info.fw_major,
                        _drone_info.fw_minor,
                        _drone_info.fw_patch,
                        std::chrono::duration_cast<std::chrono::microseconds>(
                            time.time_since_epoch() - _boot_time.time_since_epoch())
                            .count(),
                        _companion_status.dance_status,
                        _PX4_status.battery_status,
                        _PX4_status.lat,
                        _PX4_status.lon,
                        _PX4_status.alt,
                        _PX4_status.hdg,
                        _companion_status.rssi_wifi,
                        _companion_status.rssi_xbee,
                        _PX4_status.satellites_used,
                        _PX4_status.fix_type,
                        _PX4_status.mag_norm,
                        _PX4_status.alt_ref);
                    return message;
                });
            if (!result)
                LogErr() << "Unable to send drone status.";
        }
    }
}

void LumosServerImpl::set_drone_info(LumosServer::DroneInfo drone_info)
{
    std::lock_guard<std::mutex> lock(_status_mutex);
    _drone_info = drone_info;
    _info_never_set = false;
}

void LumosServerImpl::set_companion_status(LumosServer::CompanionStatus companion_status)
{
    std::lock_guard<std::mutex> lock(_status_mutex);
    _companion_status = companion_status;
}

void LumosServerImpl::battery_status_handler(const mavlink_message_t& msg)
{
    mavlink_battery_status_t status;
    mavlink_msg_battery_status_decode(&msg, &status);
    _PX4_status.battery_status = status.battery_remaining / 100.;
}

void LumosServerImpl::position_handler(const mavlink_message_t& msg)
{
    mavlink_global_position_int_t pos;
    mavlink_msg_global_position_int_decode(&msg, &pos);
    _PX4_status.lat = pos.lat;
    _PX4_status.lon = pos.lon;
    _PX4_status.alt = pos.alt;
    _PX4_status.hdg = pos.hdg;
}

void LumosServerImpl::gps_status_handler(const mavlink_message_t& msg)
{
    mavlink_gps_status_t gps;
    int sat_count = 0;

    mavlink_msg_gps_status_decode(&msg, &gps);
    for (auto i = 0; i < gps.satellites_visible && i < 20; i++) {
        if (gps.satellite_used[i] == 1)
            sat_count++;
    }
    _PX4_status.satellites_used = sat_count;
}

void LumosServerImpl::gps_raw_handler(const mavlink_message_t& msg)
{
    mavlink_gps_raw_int_t gps;

    mavlink_msg_gps_raw_int_decode(&msg, &gps);
    _PX4_status.fix_type = gps.fix_type;
}

void LumosServerImpl::highres_imu_handler(const mavlink_message_t& msg)
{
    mavlink_highres_imu_t imu;
    mavlink_msg_highres_imu_decode(&msg, &imu);
    _PX4_status.mag_norm =
        std::sqrt(imu.xmag * imu.xmag + imu.ymag * imu.ymag + imu.zmag * imu.zmag);
}

void LumosServerImpl::fram_ftp_handler(const mavlink_message_t& msg)
{
    mavlink_dance_fram_ftp_t ftp;
    mavlink_msg_dance_fram_ftp_decode(&msg, &ftp);

    auto answer = _dance_data.parse(ftp);

    auto result = _server_component_impl->queue_message([&](MavlinkAddress mavlink_address,
                                                            uint8_t channel) {
        mavlink_message_t message;
        mavlink_msg_dance_fram_ftp_encode_chan(
            mavlink_address.system_id, mavlink_address.component_id, channel, &message, &answer);
        return message;
    });
    if (!result)
        LogErr() << "Unable to send drone status.";

    if (_dance_data.is_valid()) {
        std::lock_guard<std::mutex> lock(_subscription_mutex);
        _dance_callbacks.queue(dance(), [this](const auto& func) {
            _server_component_impl->call_user_callback(func);
        });
    }
}

LumosServer::DanceHandle
LumosServerImpl::subscribe_dance(const LumosServer::DanceCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    return _dance_callbacks.subscribe(callback);
}

void LumosServerImpl::unsubscribe_dance(LumosServer::DanceHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _dance_callbacks.unsubscribe(handle);
}

LumosServer::Dance LumosServerImpl::dance()
{
    LumosServer::Dance dance;
    dance.len = 0;
    if (_dance_data.is_valid()) {
        dance.data.resize((_dance_data.size() / sizeof(uint32_t)) + 1);
        dance.len = _dance_data.size();
        memcpy(dance.data.data(), _dance_data.data(), _dance_data.size());
    }
    return dance;
}

} // namespace mavsdk
