// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see https://github.com/mavlink/MAVSDK-Proto/blob/master/protos/lumos_server/lumos_server.proto)

#include "lumos_server/lumos_server.grpc.pb.h"
#include "plugins/lumos_server/lumos_server.h"

#include "mavsdk.h"

#include "lazy_server_plugin.h"

#include "log.h"
#include <atomic>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

namespace mavsdk {
namespace mavsdk_server {

template<
    typename LumosServer = LumosServer,
    typename LazyServerPlugin = LazyServerPlugin<LumosServer>>

class LumosServerServiceImpl final : public rpc::lumos_server::LumosServerService::Service {
public:
    LumosServerServiceImpl(LazyServerPlugin& lazy_plugin) : _lazy_plugin(lazy_plugin) {}

    template<typename ResponseType>
    void fillResponseWithResult(ResponseType* response, mavsdk::LumosServer::Result& result) const
    {
        auto rpc_result = translateToRpcResult(result);

        auto* rpc_lumos_server_result = new rpc::lumos_server::LumosServerResult();
        rpc_lumos_server_result->set_result(rpc_result);
        std::stringstream ss;
        ss << result;
        rpc_lumos_server_result->set_result_str(ss.str());

        response->set_allocated_lumos_server_result(rpc_lumos_server_result);
    }

    static std::unique_ptr<rpc::lumos_server::Dance>
    translateToRpcDance(const mavsdk::LumosServer::Dance& dance)
    {
        auto rpc_obj = std::make_unique<rpc::lumos_server::Dance>();

        for (const auto& elem : dance.data) {
            rpc_obj->add_data(elem);
        }

        rpc_obj->set_len(dance.len);

        return rpc_obj;
    }

    static mavsdk::LumosServer::Dance translateFromRpcDance(const rpc::lumos_server::Dance& dance)
    {
        mavsdk::LumosServer::Dance obj;

        for (const auto& elem : dance.data()) {
            obj.data.push_back(elem);
        }

        obj.len = dance.len();

        return obj;
    }

    static std::unique_ptr<rpc::lumos_server::Position>
    translateToRpcPosition(const mavsdk::LumosServer::Position& position)
    {
        auto rpc_obj = std::make_unique<rpc::lumos_server::Position>();

        rpc_obj->set_x(position.x);

        rpc_obj->set_y(position.y);

        rpc_obj->set_z(position.z);

        return rpc_obj;
    }

    static mavsdk::LumosServer::Position
    translateFromRpcPosition(const rpc::lumos_server::Position& position)
    {
        mavsdk::LumosServer::Position obj;

        obj.x = position.x();

        obj.y = position.y();

        obj.z = position.z();

        return obj;
    }

    static std::unique_ptr<rpc::lumos_server::GlobalPosition>
    translateToRpcGlobalPosition(const mavsdk::LumosServer::GlobalPosition& global_position)
    {
        auto rpc_obj = std::make_unique<rpc::lumos_server::GlobalPosition>();

        rpc_obj->set_lat(global_position.lat);

        rpc_obj->set_lon(global_position.lon);

        rpc_obj->set_alt(global_position.alt);

        return rpc_obj;
    }

    static mavsdk::LumosServer::GlobalPosition
    translateFromRpcGlobalPosition(const rpc::lumos_server::GlobalPosition& global_position)
    {
        mavsdk::LumosServer::GlobalPosition obj;

        obj.lat = global_position.lat();

        obj.lon = global_position.lon();

        obj.alt = global_position.alt();

        return obj;
    }

    static std::unique_ptr<rpc::lumos_server::Coord>
    translateToRpcCoord(const mavsdk::LumosServer::Coord& coord)
    {
        auto rpc_obj = std::make_unique<rpc::lumos_server::Coord>();

        rpc_obj->set_x(coord.x);

        rpc_obj->set_y(coord.y);

        return rpc_obj;
    }

    static mavsdk::LumosServer::Coord translateFromRpcCoord(const rpc::lumos_server::Coord& coord)
    {
        mavsdk::LumosServer::Coord obj;

        obj.x = coord.x();

        obj.y = coord.y();

        return obj;
    }

    static std::unique_ptr<rpc::lumos_server::Params>
    translateToRpcParams(const mavsdk::LumosServer::Params& params)
    {
        auto rpc_obj = std::make_unique<rpc::lumos_server::Params>();

        rpc_obj->set_lon(params.lon);

        rpc_obj->set_lat(params.lat);

        rpc_obj->set_alt(params.alt);

        rpc_obj->set_gps_start(params.gps_start);

        rpc_obj->set_gf_alt(params.gf_alt);

        for (const auto& elem : params.vertices) {
            auto* ptr = rpc_obj->add_vertices();
            ptr->CopyFrom(*translateToRpcCoord(elem).release());
        }

        return rpc_obj;
    }

    static mavsdk::LumosServer::Params
    translateFromRpcParams(const rpc::lumos_server::Params& params)
    {
        mavsdk::LumosServer::Params obj;

        obj.lon = params.lon();

        obj.lat = params.lat();

        obj.alt = params.alt();

        obj.gps_start = params.gps_start();

        obj.gf_alt = params.gf_alt();

        for (const auto& elem : params.vertices()) {
            obj.vertices.push_back(
                translateFromRpcCoord(static_cast<mavsdk::rpc::lumos_server::Coord>(elem)));
        }

        return obj;
    }

    static rpc::lumos_server::LumosResult::Result
    translateToRpcResult(const mavsdk::LumosServer::Result& result)
    {
        switch (result) {
            default:
                LogErr() << "Unknown result enum value: " << static_cast<int>(result);
            // FALLTHROUGH
            case mavsdk::LumosServer::Result::Success:
                return rpc::lumos_server::LumosResult_Result_RESULT_SUCCESS;
            case mavsdk::LumosServer::Result::Error:
                return rpc::lumos_server::LumosResult_Result_RESULT_ERROR;
        }
    }

    static mavsdk::LumosServer::Result
    translateFromRpcResult(const rpc::lumos_server::LumosResult::Result result)
    {
        switch (result) {
            default:
                LogErr() << "Unknown result enum value: " << static_cast<int>(result);
            // FALLTHROUGH
            case rpc::lumos_server::LumosResult_Result_RESULT_SUCCESS:
                return mavsdk::LumosServer::Result::Success;
            case rpc::lumos_server::LumosResult_Result_RESULT_ERROR:
                return mavsdk::LumosServer::Result::Error;
        }
    }

    static std::unique_ptr<rpc::lumos_server::DroneInfo>
    translateToRpcDroneInfo(const mavsdk::LumosServer::DroneInfo& drone_info)
    {
        auto rpc_obj = std::make_unique<rpc::lumos_server::DroneInfo>();

        rpc_obj->set_uuid(drone_info.uuid);

        rpc_obj->set_fw_major(drone_info.fw_major);

        rpc_obj->set_fw_minor(drone_info.fw_minor);

        rpc_obj->set_fw_patch(drone_info.fw_patch);

        return rpc_obj;
    }

    static mavsdk::LumosServer::DroneInfo
    translateFromRpcDroneInfo(const rpc::lumos_server::DroneInfo& drone_info)
    {
        mavsdk::LumosServer::DroneInfo obj;

        obj.uuid = drone_info.uuid();

        obj.fw_major = drone_info.fw_major();

        obj.fw_minor = drone_info.fw_minor();

        obj.fw_patch = drone_info.fw_patch();

        return obj;
    }

    static std::unique_ptr<rpc::lumos_server::CompanionStatus>
    translateToRpcCompanionStatus(const mavsdk::LumosServer::CompanionStatus& companion_status)
    {
        auto rpc_obj = std::make_unique<rpc::lumos_server::CompanionStatus>();

        rpc_obj->set_dance_status(companion_status.dance_status);

        rpc_obj->set_rssi_wifi(companion_status.rssi_wifi);

        rpc_obj->set_rssi_xbee(companion_status.rssi_xbee);

        rpc_obj->set_battery_status(companion_status.battery_status);

        return rpc_obj;
    }

    static mavsdk::LumosServer::CompanionStatus
    translateFromRpcCompanionStatus(const rpc::lumos_server::CompanionStatus& companion_status)
    {
        mavsdk::LumosServer::CompanionStatus obj;

        obj.dance_status = companion_status.dance_status();

        obj.rssi_wifi = companion_status.rssi_wifi();

        obj.rssi_xbee = companion_status.rssi_xbee();

        obj.battery_status = companion_status.battery_status();

        return obj;
    }

    static std::unique_ptr<rpc::lumos_server::LedInfo>
    translateToRpcLedInfo(const mavsdk::LumosServer::LedInfo& led_info)
    {
        auto rpc_obj = std::make_unique<rpc::lumos_server::LedInfo>();

        rpc_obj->set_color(led_info.color);

        rpc_obj->set_mode(led_info.mode);

        rpc_obj->set_blink_count(led_info.blink_count);

        rpc_obj->set_prio(led_info.prio);

        return rpc_obj;
    }

    static mavsdk::LumosServer::LedInfo
    translateFromRpcLedInfo(const rpc::lumos_server::LedInfo& led_info)
    {
        mavsdk::LumosServer::LedInfo obj;

        obj.color = led_info.color();

        obj.mode = led_info.mode();

        obj.blink_count = led_info.blink_count();

        obj.prio = led_info.prio();

        return obj;
    }

    grpc::Status SetDroneInfo(
        grpc::ServerContext* /* context */,
        const rpc::lumos_server::SetDroneInfoRequest* request,
        rpc::lumos_server::SetDroneInfoResponse* /* response */) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            return grpc::Status::OK;
        }

        if (request == nullptr) {
            LogWarn() << "SetDroneInfo sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        _lazy_plugin.maybe_plugin()->set_drone_info(
            translateFromRpcDroneInfo(request->drone_info()));

        return grpc::Status::OK;
    }

    grpc::Status SetCompanionStatus(
        grpc::ServerContext* /* context */,
        const rpc::lumos_server::SetCompanionStatusRequest* request,
        rpc::lumos_server::SetCompanionStatusResponse* /* response */) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            return grpc::Status::OK;
        }

        if (request == nullptr) {
            LogWarn() << "SetCompanionStatus sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        _lazy_plugin.maybe_plugin()->set_companion_status(
            translateFromRpcCompanionStatus(request->drone_info()));

        return grpc::Status::OK;
    }

    grpc::Status SubscribeDance(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::lumos_server::SubscribeDanceRequest* /* request */,
        grpc::ServerWriter<rpc::lumos_server::DanceResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        const mavsdk::LumosServer::DanceHandle handle =
            _lazy_plugin.maybe_plugin()->subscribe_dance(
                [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex, &handle](
                    const mavsdk::LumosServer::Dance dance) {
                    rpc::lumos_server::DanceResponse rpc_response;

                    rpc_response.set_allocated_dance(translateToRpcDance(dance).release());

                    std::unique_lock<std::mutex> lock(*subscribe_mutex);
                    if (!*is_finished && !writer->Write(rpc_response)) {
                        _lazy_plugin.maybe_plugin()->unsubscribe_dance(handle);

                        *is_finished = true;
                        unregister_stream_stop_promise(stream_closed_promise);
                        stream_closed_promise->set_value();
                    }
                });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

        return grpc::Status::OK;
    }

    grpc::Status SubscribeParams(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::lumos_server::SubscribeParamsRequest* /* request */,
        grpc::ServerWriter<rpc::lumos_server::ParamsResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        const mavsdk::LumosServer::ParamsHandle handle =
            _lazy_plugin.maybe_plugin()->subscribe_params(
                [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex, &handle](
                    const mavsdk::LumosServer::Params params) {
                    rpc::lumos_server::ParamsResponse rpc_response;

                    rpc_response.set_allocated_params(translateToRpcParams(params).release());

                    std::unique_lock<std::mutex> lock(*subscribe_mutex);
                    if (!*is_finished && !writer->Write(rpc_response)) {
                        _lazy_plugin.maybe_plugin()->unsubscribe_params(handle);

                        *is_finished = true;
                        unregister_stream_stop_promise(stream_closed_promise);
                        stream_closed_promise->set_value();
                    }
                });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

        return grpc::Status::OK;
    }

    grpc::Status SubscribeStart(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::lumos_server::SubscribeStartRequest* /* request */,
        grpc::ServerWriter<rpc::lumos_server::StartResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        const mavsdk::LumosServer::StartHandle handle =
            _lazy_plugin.maybe_plugin()->subscribe_start(
                [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex, &handle](
                    const int32_t start) {
                    rpc::lumos_server::StartResponse rpc_response;

                    rpc_response.set_mode(start);

                    std::unique_lock<std::mutex> lock(*subscribe_mutex);
                    if (!*is_finished && !writer->Write(rpc_response)) {
                        _lazy_plugin.maybe_plugin()->unsubscribe_start(handle);

                        *is_finished = true;
                        unregister_stream_stop_promise(stream_closed_promise);
                        stream_closed_promise->set_value();
                    }
                });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

        return grpc::Status::OK;
    }

    grpc::Status SubscribeLocalPos(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::lumos_server::SubscribeLocalPosRequest* /* request */,
        grpc::ServerWriter<rpc::lumos_server::LocalPosResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        const mavsdk::LumosServer::LocalPosHandle handle =
            _lazy_plugin.maybe_plugin()->subscribe_local_pos(
                [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex, &handle](
                    const mavsdk::LumosServer::Position local_pos) {
                    rpc::lumos_server::LocalPosResponse rpc_response;

                    rpc_response.set_allocated_pos(translateToRpcPosition(local_pos).release());

                    std::unique_lock<std::mutex> lock(*subscribe_mutex);
                    if (!*is_finished && !writer->Write(rpc_response)) {
                        _lazy_plugin.maybe_plugin()->unsubscribe_local_pos(handle);

                        *is_finished = true;
                        unregister_stream_stop_promise(stream_closed_promise);
                        stream_closed_promise->set_value();
                    }
                });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

        return grpc::Status::OK;
    }

    grpc::Status SubscribeGlobalPos(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::lumos_server::SubscribeGlobalPosRequest* /* request */,
        grpc::ServerWriter<rpc::lumos_server::GlobalPosResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        const mavsdk::LumosServer::GlobalPosHandle handle =
            _lazy_plugin.maybe_plugin()->subscribe_global_pos(
                [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex, &handle](
                    const mavsdk::LumosServer::GlobalPosition global_pos) {
                    rpc::lumos_server::GlobalPosResponse rpc_response;

                    rpc_response.set_allocated_pos(
                        translateToRpcGlobalPosition(global_pos).release());

                    std::unique_lock<std::mutex> lock(*subscribe_mutex);
                    if (!*is_finished && !writer->Write(rpc_response)) {
                        _lazy_plugin.maybe_plugin()->unsubscribe_global_pos(handle);

                        *is_finished = true;
                        unregister_stream_stop_promise(stream_closed_promise);
                        stream_closed_promise->set_value();
                    }
                });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

        return grpc::Status::OK;
    }

    grpc::Status SubscribeLandCmd(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::lumos_server::SubscribeLandCmdRequest* /* request */,
        grpc::ServerWriter<rpc::lumos_server::LandCmdResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        const mavsdk::LumosServer::LandCmdHandle handle =
            _lazy_plugin.maybe_plugin()->subscribe_land_cmd(
                [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex, &handle](
                    const int32_t land_cmd) {
                    rpc::lumos_server::LandCmdResponse rpc_response;

                    rpc_response.set_unused(land_cmd);

                    std::unique_lock<std::mutex> lock(*subscribe_mutex);
                    if (!*is_finished && !writer->Write(rpc_response)) {
                        _lazy_plugin.maybe_plugin()->unsubscribe_land_cmd(handle);

                        *is_finished = true;
                        unregister_stream_stop_promise(stream_closed_promise);
                        stream_closed_promise->set_value();
                    }
                });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

        return grpc::Status::OK;
    }

    grpc::Status SubscribeRtlCmd(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::lumos_server::SubscribeRtlCmdRequest* /* request */,
        grpc::ServerWriter<rpc::lumos_server::RtlCmdResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        const mavsdk::LumosServer::RtlCmdHandle handle =
            _lazy_plugin.maybe_plugin()->subscribe_rtl_cmd(
                [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex, &handle](
                    const int32_t rtl_cmd) {
                    rpc::lumos_server::RtlCmdResponse rpc_response;

                    rpc_response.set_unused(rtl_cmd);

                    std::unique_lock<std::mutex> lock(*subscribe_mutex);
                    if (!*is_finished && !writer->Write(rpc_response)) {
                        _lazy_plugin.maybe_plugin()->unsubscribe_rtl_cmd(handle);

                        *is_finished = true;
                        unregister_stream_stop_promise(stream_closed_promise);
                        stream_closed_promise->set_value();
                    }
                });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

        return grpc::Status::OK;
    }

    grpc::Status SubscribeKillCmd(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::lumos_server::SubscribeKillCmdRequest* /* request */,
        grpc::ServerWriter<rpc::lumos_server::KillCmdResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        const mavsdk::LumosServer::KillCmdHandle handle =
            _lazy_plugin.maybe_plugin()->subscribe_kill_cmd(
                [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex, &handle](
                    const int32_t kill_cmd) {
                    rpc::lumos_server::KillCmdResponse rpc_response;

                    rpc_response.set_unused(kill_cmd);

                    std::unique_lock<std::mutex> lock(*subscribe_mutex);
                    if (!*is_finished && !writer->Write(rpc_response)) {
                        _lazy_plugin.maybe_plugin()->unsubscribe_kill_cmd(handle);

                        *is_finished = true;
                        unregister_stream_stop_promise(stream_closed_promise);
                        stream_closed_promise->set_value();
                    }
                });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

        return grpc::Status::OK;
    }

    grpc::Status SubscribeColorLedCmd(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::lumos_server::SubscribeColorLedCmdRequest* /* request */,
        grpc::ServerWriter<rpc::lumos_server::ColorLedCmdResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        const mavsdk::LumosServer::ColorLedCmdHandle handle =
            _lazy_plugin.maybe_plugin()->subscribe_color_led_cmd(
                [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex, &handle](
                    const mavsdk::LumosServer::LedInfo color_led_cmd) {
                    rpc::lumos_server::ColorLedCmdResponse rpc_response;

                    rpc_response.set_allocated_led(translateToRpcLedInfo(color_led_cmd).release());

                    std::unique_lock<std::mutex> lock(*subscribe_mutex);
                    if (!*is_finished && !writer->Write(rpc_response)) {
                        _lazy_plugin.maybe_plugin()->unsubscribe_color_led_cmd(handle);

                        *is_finished = true;
                        unregister_stream_stop_promise(stream_closed_promise);
                        stream_closed_promise->set_value();
                    }
                });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

        return grpc::Status::OK;
    }

    void stop()
    {
        _stopped.store(true);
        for (auto& prom : _stream_stop_promises) {
            if (auto handle = prom.lock()) {
                handle->set_value();
            }
        }
    }

private:
    void register_stream_stop_promise(std::weak_ptr<std::promise<void>> prom)
    {
        // If we have already stopped, set promise immediately and don't add it to list.
        if (_stopped.load()) {
            if (auto handle = prom.lock()) {
                handle->set_value();
            }
        } else {
            _stream_stop_promises.push_back(prom);
        }
    }

    void unregister_stream_stop_promise(std::shared_ptr<std::promise<void>> prom)
    {
        for (auto it = _stream_stop_promises.begin(); it != _stream_stop_promises.end();
             /* ++it */) {
            if (it->lock() == prom) {
                it = _stream_stop_promises.erase(it);
            } else {
                ++it;
            }
        }
    }

    LazyServerPlugin& _lazy_plugin;

    std::atomic<bool> _stopped{false};
    std::vector<std::weak_ptr<std::promise<void>>> _stream_stop_promises{};
};

} // namespace mavsdk_server
} // namespace mavsdk