#pragma once

#include <thread>

#include "plugins/lumos_server/lumos_server.h"
#include "callback_list.tpp"
#include "server_plugin_impl_base.h"

#include "dance_data.h"

namespace mavsdk {

class LumosServerImpl : public ServerPluginImplBase {
public:
    explicit LumosServerImpl(std::shared_ptr<ServerComponent> server_component);

    ~LumosServerImpl() override;

    void init() override;
    void deinit() override;

    void set_drone_info(LumosServer::DroneInfo drone_info);
    void set_companion_status(LumosServer::CompanionStatus companion_status);
    LumosServer::DanceHandle subscribe_dance(const LumosServer::DanceCallback& callback);
    void unsubscribe_dance(LumosServer::DanceHandle handle);
    LumosServer::Dance dance();
    LumosServer::ParamsHandle subscribe_params(const LumosServer::ParamsCallback& callback);
    void unsubscribe_params(LumosServer::ParamsHandle handle);
    LumosServer::Params params();
    LumosServer::StartHandle subscribe_start(const LumosServer::StartCallback& callback);
    void unsubscribe_start(LumosServer::StartHandle handle);
    int32_t start();
    LumosServer::LocalPosHandle subscribe_local_pos(const LumosServer::LocalPosCallback& callback);
    void unsubscribe_local_pos(LumosServer::LocalPosHandle handle);
    LumosServer::Position local_pos();
    LumosServer::GlobalPosHandle
    subscribe_global_pos(const LumosServer::GlobalPosCallback& callback);
    void unsubscribe_global_pos(LumosServer::GlobalPosHandle handle);
    LumosServer::GlobalPosition global_pos();

private:
    struct PX4Status {
        float battery_status;
        int32_t lat;
        int32_t lon;
        int32_t alt;
        uint16_t hdg;
        uint8_t satellites_used;
        uint8_t fix_type;
        float mag_norm;
        uint8_t alt_ref{1};
    };

    void drone_status_thread();

    void battery_status_handler(const mavlink_message_t& msg);
    void global_position_handler(const mavlink_message_t& msg);
    void gps_status_handler(const mavlink_message_t& msg);
    void gps_raw_handler(const mavlink_message_t& msg);
    void highres_imu_handler(const mavlink_message_t& msg);
    void fram_ftp_handler(const mavlink_message_t& msg);
    void local_position_ned_handler(const mavlink_message_t& msg);
    std::optional<mavlink_command_ack_t>
    do_set_mode_handler(const MavlinkCommandReceiver::CommandLong& command);

    LumosServer::DroneInfo _drone_info;
    bool _info_never_set{true};

    LumosServer::CompanionStatus _companion_status;

    std::mutex _status_mutex;
    std::thread _status_thread;
    bool _stop_threads{false};

    PX4Status _PX4_status;

    std::chrono::time_point<std::chrono::steady_clock> _boot_time;

    DanceData _dance_data;

    mavlink_local_position_ned_t _local_pos_ned{};

    std::mutex _subscription_mutex{};
    CallbackList<LumosServer::Dance> _dance_callbacks;
    CallbackList<LumosServer::Params> _params_callbacks;
    CallbackList<int32_t> _start_callbacks;
    CallbackList<LumosServer::Position> _local_pos_callbacks;
    CallbackList<LumosServer::GlobalPosition> _global_pos_callbacks;
};

} // namespace mavsdk
