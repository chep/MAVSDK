#pragma once

#include <thread>

#include "plugins/lumos_server/lumos_server.h"

#include "server_plugin_impl_base.h"

namespace mavsdk {

class LumosServerImpl : public ServerPluginImplBase {
public:
    explicit LumosServerImpl(std::shared_ptr<ServerComponent> server_component);

    ~LumosServerImpl() override;

    void init() override;
    void deinit() override;

    void set_drone_status(LumosServer::DroneStatus drone_status);

private:
    void drone_status_thread();

    LumosServer::DroneStatus _status;
    bool _status_never_set{true};
    std::mutex _status_mutex;
    std::thread _status_thread;
    bool _stop_threads{false};
};

} // namespace mavsdk
