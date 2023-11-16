#include "lumos_server_impl.h"

namespace mavsdk {

static const std::chrono::milliseconds STATUS_MESSAGE_INTERVAL(1000);

LumosServerImpl::LumosServerImpl(std::shared_ptr<ServerComponent> server_component) :
    ServerPluginImplBase(server_component)
{
    _server_component_impl->register_plugin(this);
}

LumosServerImpl::~LumosServerImpl()
{
    _server_component_impl->unregister_plugin(this);
}

void LumosServerImpl::init()
{
    _status_thread = std::thread(std::bind(&LumosServerImpl::drone_status_thread, this));
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
        if (!_status_never_set) {
            std::lock_guard<std::mutex> lock(_status_mutex);
            LogDebug() << "Sending drone status";
            auto result = _server_component_impl->queue_message([&](MavlinkAddress mavlink_address,
                                                                    uint8_t channel) {
                mavlink_message_t message;
                char uuid[12];
                std::memcpy(uuid, _status.uuid.c_str(), sizeof(uuid));
                const auto time = std::chrono::system_clock::now();
                mavlink_msg_drone_status_pack_chan(
                    mavlink_address.system_id,
                    mavlink_address.component_id,
                    channel,
                    &message,
                    reinterpret_cast<uint8_t*>(uuid),
                    _status.fw_major,
                    _status.fw_minor,
                    _status.fw_patch,
                    std::chrono::duration_cast<std::chrono::microseconds>(time.time_since_epoch())
                        .count(),
                    _status.dance_status,
                    _status.battery_status,
                    _status.lat,
                    _status.lon,
                    _status.alt,
                    _status.hdg,
                    _status.rssi_wifi,
                    _status.rssi_xbee,
                    _status.satellites_used,
                    _status.fix_type,
                    _status.mag_norm,
                    _status.alt_ref);
                return message;
            });
            if (!result)
                LogErr() << "Unable to send drone status.";
        }
    }
}

void LumosServerImpl::set_drone_status(LumosServer::DroneStatus drone_status)
{
    std::lock_guard<std::mutex> lock(_status_mutex);
    _status = drone_status;
    _status_never_set = false;
}

} // namespace mavsdk
