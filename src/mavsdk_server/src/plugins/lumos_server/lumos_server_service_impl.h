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

        return rpc_obj;
    }

    static mavsdk::LumosServer::CompanionStatus
    translateFromRpcCompanionStatus(const rpc::lumos_server::CompanionStatus& companion_status)
    {
        mavsdk::LumosServer::CompanionStatus obj;

        obj.dance_status = companion_status.dance_status();

        obj.rssi_wifi = companion_status.rssi_wifi();

        obj.rssi_xbee = companion_status.rssi_xbee();

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