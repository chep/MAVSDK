#include "lumos_server_impl.h"

namespace mavsdk {

static const std::chrono::milliseconds STATUS_MESSAGE_INTERVAL(1000);
constexpr int PX4_MODE_UNKNOWN = 1;
constexpr int PX4_CUSTOM_MODE_OFFBOARD = 6;

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
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        std::bind(&LumosServerImpl::global_position_handler, this, std::placeholders::_1),
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
    _server_component_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        std::bind(&LumosServerImpl::local_position_ned_handler, this, std::placeholders::_1),
        nullptr);
    _server_component_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_GPS_RTCM_DATA,
        std::bind(&LumosServerImpl::gps_rtcm_data_handler, this, std::placeholders::_1),
        nullptr);

    // GCS messages handlers
    _server_component_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_DANCE_FRAM_FTP,
        std::bind(&LumosServerImpl::fram_ftp_handler, this, std::placeholders::_1),
        nullptr);

    // GCS commands
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_DO_SET_MODE,
        std::bind(&LumosServerImpl::do_set_mode_handler, this, std::placeholders::_1),
        nullptr);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_NAV_LAND,
        std::bind(&LumosServerImpl::nav_land_handler, this, std::placeholders::_1),
        nullptr);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_NAV_RETURN_TO_LAUNCH,
        std::bind(&LumosServerImpl::rtl_handler, this, std::placeholders::_1),
        nullptr);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_COLOR_LED,
        std::bind(&LumosServerImpl::color_led_handler, this, std::placeholders::_1),
        nullptr);

    // GCS messages
    _server_component_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_FLIGHT_TERMINATION,
        std::bind(&LumosServerImpl::kill_handler, this, std::placeholders::_1),
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
                        _companion_status.battery_status,
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

void LumosServerImpl::global_position_handler(const mavlink_message_t& msg)
{
    mavlink_global_position_int_t pos;
    mavlink_msg_global_position_int_decode(&msg, &pos);
    _PX4_status.lat = pos.lat;
    _PX4_status.lon = pos.lon;
    _PX4_status.alt = pos.alt;
    _PX4_status.hdg = pos.hdg;

    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _global_pos_callbacks.queue(global_pos(), [this](const auto& func) {
        _server_component_impl->call_user_callback(func);
    });
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

    if (answer.payload[0] == DanceData::ACK) {
        switch (answer.opcode) {
            case MAV_DANCE_FRAM_OPCODES_CRC32:
                if (_dance_data.is_valid()) {
                    std::lock_guard<std::mutex> lock(_subscription_mutex);
                    _dance_callbacks.queue(dance(), [this](const auto& func) {
                        _server_component_impl->call_user_callback(func);
                    });
                    _dance_data.clear();
                }
                break;
            case MAV_DANCE_FRAM_OPCODES_PARAMS: {
                std::lock_guard<std::mutex> lock(_subscription_mutex);
                _params_callbacks.queue(params(), [this](const auto& func) {
                    _server_component_impl->call_user_callback(func);
                });
            } break;
            default:
                break;
        }
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

LumosServer::ParamsHandle
LumosServerImpl::subscribe_params(const LumosServer::ParamsCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    return _params_callbacks.subscribe(callback);
}

void LumosServerImpl::unsubscribe_params(LumosServer::ParamsHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _params_callbacks.unsubscribe(handle);
}

LumosServer::Params LumosServerImpl::params()
{
    LumosServer::Params params;

    auto p = _dance_data.params();
    params.alt = p.alt;
    params.gf_alt = p.gf_alt;
    params.gps_start = p.gps_start;
    params.lat = p.lat;
    params.lon = p.lon;

    for (const auto& poly : p.poly) {
        LumosServer::Coord c;
        c.x = poly.x;
        c.y = poly.y;
        params.vertices.push_back(c);
    }

    return params;
}

LumosServer::StartHandle
LumosServerImpl::subscribe_start(const LumosServer::StartCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    return _start_callbacks.subscribe(callback);
}

void LumosServerImpl::unsubscribe_start(LumosServer::StartHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _start_callbacks.unsubscribe(handle);
}

int32_t LumosServerImpl::start()
{
    return 0;
}

std::optional<mavlink_command_ack_t>
LumosServerImpl::do_set_mode_handler(const MavlinkCommandReceiver::CommandLong& command)
{
    if (int(command.params.param1) == PX4_MODE_UNKNOWN and
        int(command.params.param2) == PX4_CUSTOM_MODE_OFFBOARD) {
        std::lock_guard<std::mutex> lock(_subscription_mutex);
        _start_callbacks.queue(command.params.param1, [this](const auto& func) {
            _server_component_impl->call_user_callback(func);
        });
    }

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

LumosServer::LocalPosHandle
LumosServerImpl::subscribe_local_pos(const LumosServer::LocalPosCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    return _local_pos_callbacks.subscribe(callback);
}

void LumosServerImpl::unsubscribe_local_pos(LumosServer::LocalPosHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _local_pos_callbacks.unsubscribe(handle);
}

LumosServer::Position LumosServerImpl::local_pos()
{
    LumosServer::Position pos{_local_pos_ned.x, _local_pos_ned.y, _local_pos_ned.z};
    return pos;
}

void LumosServerImpl::local_position_ned_handler(const mavlink_message_t& msg)
{
    mavlink_msg_local_position_ned_decode(&msg, &_local_pos_ned);
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _local_pos_callbacks.queue(local_pos(), [this](const auto& func) {
        _server_component_impl->call_user_callback(func);
    });
}

void LumosServerImpl::gps_rtcm_data_handler(const mavlink_message_t& msg)
{
    auto result = _server_component_impl->queue_message(
        [&]([[maybe_unused]] MavlinkAddress mavlink_address, [[maybe_unused]] uint8_t channel) {
            return msg;
        });
    if (!result)
        LogErr() << "Unable to forward rtcm data.";
}

LumosServer::GlobalPosHandle
LumosServerImpl::subscribe_global_pos(const LumosServer::GlobalPosCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    return _global_pos_callbacks.subscribe(callback);
}

void LumosServerImpl::unsubscribe_global_pos(LumosServer::GlobalPosHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _global_pos_callbacks.unsubscribe(handle);
}

LumosServer::GlobalPosition LumosServerImpl::global_pos()
{
    LumosServer::GlobalPosition pos{
        double(_PX4_status.lat) / 1E7,
        double(_PX4_status.lon) / 1E7,
        float(_PX4_status.alt) / 1000.f};
    return pos;
}

LumosServer::LandCmdHandle
LumosServerImpl::subscribe_land_cmd(const LumosServer::LandCmdCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    return _land_cmd_callbacks.subscribe(callback);
}

void LumosServerImpl::unsubscribe_land_cmd(LumosServer::LandCmdHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _land_cmd_callbacks.unsubscribe(handle);
}

int32_t LumosServerImpl::land_cmd()
{
    return 0;
}

std::optional<mavlink_command_ack_t>
LumosServerImpl::nav_land_handler(const MavlinkCommandReceiver::CommandLong& command)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _land_cmd_callbacks.queue(
        0, [this](const auto& func) { _server_component_impl->call_user_callback(func); });

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

LumosServer::RtlCmdHandle
LumosServerImpl::subscribe_rtl_cmd(const LumosServer::RtlCmdCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    return _rtl_cmd_callbacks.subscribe(callback);
}

void LumosServerImpl::unsubscribe_rtl_cmd(LumosServer::RtlCmdHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _rtl_cmd_callbacks.unsubscribe(handle);
}

int32_t LumosServerImpl::rtl_cmd()
{
    return 0;
}

std::optional<mavlink_command_ack_t>
LumosServerImpl::rtl_handler(const MavlinkCommandReceiver::CommandLong& command)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _rtl_cmd_callbacks.queue(
        0, [this](const auto& func) { _server_component_impl->call_user_callback(func); });

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

LumosServer::ColorLedCmdHandle
LumosServerImpl::subscribe_color_led_cmd(const LumosServer::ColorLedCmdCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    return _color_led_cmd_callbacks.subscribe(callback);
}

void LumosServerImpl::unsubscribe_color_led_cmd(LumosServer::ColorLedCmdHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _color_led_cmd_callbacks.unsubscribe(handle);
}

LumosServer::LedInfo LumosServerImpl::color_led_cmd()
{
    return _last_led_info;
}

std::optional<mavlink_command_ack_t>
LumosServerImpl::color_led_handler(const MavlinkCommandReceiver::CommandLong& command)
{
    _last_led_info.color = command.params.param1;
    _last_led_info.mode = command.params.param2;
    _last_led_info.blink_count = command.params.param3;
    _last_led_info.prio = command.params.param4;

    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _color_led_cmd_callbacks.queue(_last_led_info, [this](const auto& func) {
        _server_component_impl->call_user_callback(func);
    });

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

LumosServer::KillCmdHandle
LumosServerImpl::subscribe_kill_cmd(const LumosServer::KillCmdCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    return _kill_cmd_callbacks.subscribe(callback);
}

void LumosServerImpl::unsubscribe_kill_cmd(LumosServer::KillCmdHandle handle)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _kill_cmd_callbacks.unsubscribe(handle);
}

int32_t LumosServerImpl::kill_cmd()
{
    return 0;
}

void LumosServerImpl::kill_handler([[maybe_unused]] const mavlink_message_t& msg)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _kill_cmd_callbacks.queue(
        0, [this](const auto& func) { _server_component_impl->call_user_callback(func); });
}

} // namespace mavsdk
