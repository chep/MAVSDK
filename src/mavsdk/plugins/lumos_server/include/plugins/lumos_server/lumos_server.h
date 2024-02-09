// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see https://github.com/mavlink/MAVSDK-Proto/blob/main/protos/lumos_server/lumos_server.proto)

#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "server_plugin_base.h"

#include "handle.h"

namespace mavsdk {

class ServerComponent;
class LumosServerImpl;

/**
 * @brief
 */
class LumosServer : public ServerPluginBase {
public:
    /**
     * @brief Constructor. Creates the plugin for a ServerComponent instance.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto lumos_server = LumosServer(server_component);
     *     ```
     *
     * @param server_component The ServerComponent instance associated with this server plugin.
     */
    explicit LumosServer(std::shared_ptr<ServerComponent> server_component);

    /**
     * @brief Destructor (internal use only).
     */
    ~LumosServer() override;

    /**
     * @brief
     */
    struct Dance {
        std::vector<uint32_t> data{}; /**< @brief */
        uint32_t len{}; /**< @brief */
    };

    /**
     * @brief Equal operator to compare two `LumosServer::Dance` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const LumosServer::Dance& lhs, const LumosServer::Dance& rhs);

    /**
     * @brief Stream operator to print information about a `LumosServer::Dance`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, LumosServer::Dance const& dance);

    /**
     * @brief
     */
    struct Position {
        float x{}; /**< @brief */
        float y{}; /**< @brief */
        float z{}; /**< @brief */
    };

    /**
     * @brief Equal operator to compare two `LumosServer::Position` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const LumosServer::Position& lhs, const LumosServer::Position& rhs);

    /**
     * @brief Stream operator to print information about a `LumosServer::Position`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, LumosServer::Position const& position);

    /**
     * @brief
     */
    struct GlobalPosition {
        double lat{}; /**< @brief */
        double lon{}; /**< @brief */
        float alt{}; /**< @brief */
    };

    /**
     * @brief Equal operator to compare two `LumosServer::GlobalPosition` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool
    operator==(const LumosServer::GlobalPosition& lhs, const LumosServer::GlobalPosition& rhs);

    /**
     * @brief Stream operator to print information about a `LumosServer::GlobalPosition`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream&
    operator<<(std::ostream& str, LumosServer::GlobalPosition const& global_position);

    /**
     * @brief
     */
    struct Coord {
        float x{}; /**< @brief */
        float y{}; /**< @brief */
    };

    /**
     * @brief Equal operator to compare two `LumosServer::Coord` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const LumosServer::Coord& lhs, const LumosServer::Coord& rhs);

    /**
     * @brief Stream operator to print information about a `LumosServer::Coord`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, LumosServer::Coord const& coord);

    /**
     * @brief
     */
    struct Params {
        float lon{}; /**< @brief */
        float lat{}; /**< @brief */
        float alt{}; /**< @brief */
        int32_t gps_start{}; /**< @brief */
        float gf_alt{}; /**< @brief */
        std::vector<Coord> vertices{}; /**< @brief */
    };

    /**
     * @brief Equal operator to compare two `LumosServer::Params` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const LumosServer::Params& lhs, const LumosServer::Params& rhs);

    /**
     * @brief Stream operator to print information about a `LumosServer::Params`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, LumosServer::Params const& params);

    /**
     * @brief Possible results returned for commands
     */
    enum class Result {
        Success, /**< @brief. */
        Error, /**< @brief. */
    };

    /**
     * @brief Stream operator to print information about a `LumosServer::Result`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, LumosServer::Result const& result);

    /**
     * @brief
     */
    struct DroneInfo {
        std::string uuid{}; /**< @brief<  uuid */
        uint32_t fw_major{}; /**< @brief<  Firmware version major (first byte) */
        uint32_t fw_minor{}; /**< @brief<  Firmware version minor (second byte) */
        uint32_t fw_patch{}; /**< @brief<  Firmware version patch (third byte) */
    };

    /**
     * @brief Equal operator to compare two `LumosServer::DroneInfo` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const LumosServer::DroneInfo& lhs, const LumosServer::DroneInfo& rhs);

    /**
     * @brief Stream operator to print information about a `LumosServer::DroneInfo`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, LumosServer::DroneInfo const& drone_info);

    /**
     * @brief
     */
    struct CompanionStatus {
        uint32_t dance_status{}; /**< @brief */
        uint32_t rssi_wifi{}; /**< @brief */
        uint32_t rssi_xbee{}; /**< @brief */
    };

    /**
     * @brief Equal operator to compare two `LumosServer::CompanionStatus` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool
    operator==(const LumosServer::CompanionStatus& lhs, const LumosServer::CompanionStatus& rhs);

    /**
     * @brief Stream operator to print information about a `LumosServer::CompanionStatus`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream&
    operator<<(std::ostream& str, LumosServer::CompanionStatus const& companion_status);

    /**
     * @brief
     */
    struct LedInfo {
        uint32_t color{}; /**< @brief */
        uint32_t mode{}; /**< @brief */
        uint32_t blink_count{}; /**< @brief */
        uint32_t prio{}; /**< @brief */
    };

    /**
     * @brief Equal operator to compare two `LumosServer::LedInfo` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const LumosServer::LedInfo& lhs, const LumosServer::LedInfo& rhs);

    /**
     * @brief Stream operator to print information about a `LumosServer::LedInfo`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, LumosServer::LedInfo const& led_info);

    /**
     * @brief Callback type for asynchronous LumosServer calls.
     */
    using ResultCallback = std::function<void(Result)>;

    /**
     * @brief
     *
     * This function is blocking.
     *
     * @return Result of request.
     */
    void set_drone_info(DroneInfo drone_info) const;

    /**
     * @brief
     *
     * This function is blocking.
     *
     * @return Result of request.
     */
    void set_companion_status(CompanionStatus drone_info) const;

    /**
     * @brief Callback type for subscribe_dance.
     */
    using DanceCallback = std::function<void(Dance)>;

    /**
     * @brief Handle type for subscribe_dance.
     */
    using DanceHandle = Handle<Dance>;

    /**
     * @brief
     */
    DanceHandle subscribe_dance(const DanceCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_dance
     */
    void unsubscribe_dance(DanceHandle handle);

    /**
     * @brief Poll for 'Dance' (blocking).
     *
     * @return One Dance update.
     */
    Dance dance() const;

    /**
     * @brief Callback type for subscribe_params.
     */
    using ParamsCallback = std::function<void(Params)>;

    /**
     * @brief Handle type for subscribe_params.
     */
    using ParamsHandle = Handle<Params>;

    /**
     * @brief
     */
    ParamsHandle subscribe_params(const ParamsCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_params
     */
    void unsubscribe_params(ParamsHandle handle);

    /**
     * @brief Poll for 'Params' (blocking).
     *
     * @return One Params update.
     */
    Params params() const;

    /**
     * @brief Callback type for subscribe_start.
     */
    using StartCallback = std::function<void(int32_t)>;

    /**
     * @brief Handle type for subscribe_start.
     */
    using StartHandle = Handle<int32_t>;

    /**
     * @brief
     */
    StartHandle subscribe_start(const StartCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_start
     */
    void unsubscribe_start(StartHandle handle);

    /**
     * @brief Poll for 'int32_t' (blocking).
     *
     * @return One int32_t update.
     */
    int32_t start() const;

    /**
     * @brief Callback type for subscribe_local_pos.
     */
    using LocalPosCallback = std::function<void(Position)>;

    /**
     * @brief Handle type for subscribe_local_pos.
     */
    using LocalPosHandle = Handle<Position>;

    /**
     * @brief
     */
    LocalPosHandle subscribe_local_pos(const LocalPosCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_local_pos
     */
    void unsubscribe_local_pos(LocalPosHandle handle);

    /**
     * @brief Poll for 'Position' (blocking).
     *
     * @return One Position update.
     */
    Position local_pos() const;

    /**
     * @brief Callback type for subscribe_global_pos.
     */
    using GlobalPosCallback = std::function<void(GlobalPosition)>;

    /**
     * @brief Handle type for subscribe_global_pos.
     */
    using GlobalPosHandle = Handle<GlobalPosition>;

    /**
     * @brief
     */
    GlobalPosHandle subscribe_global_pos(const GlobalPosCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_global_pos
     */
    void unsubscribe_global_pos(GlobalPosHandle handle);

    /**
     * @brief Poll for 'GlobalPosition' (blocking).
     *
     * @return One GlobalPosition update.
     */
    GlobalPosition global_pos() const;

    /**
     * @brief Callback type for subscribe_land_cmd.
     */
    using LandCmdCallback = std::function<void(int32_t)>;

    /**
     * @brief Handle type for subscribe_land_cmd.
     */
    using LandCmdHandle = Handle<int32_t>;

    /**
     * @brief
     */
    LandCmdHandle subscribe_land_cmd(const LandCmdCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_land_cmd
     */
    void unsubscribe_land_cmd(LandCmdHandle handle);

    /**
     * @brief Poll for 'int32_t' (blocking).
     *
     * @return One int32_t update.
     */
    int32_t land_cmd() const;

    /**
     * @brief Callback type for subscribe_rtl_cmd.
     */
    using RtlCmdCallback = std::function<void(int32_t)>;

    /**
     * @brief Handle type for subscribe_rtl_cmd.
     */
    using RtlCmdHandle = Handle<int32_t>;

    /**
     * @brief
     */
    RtlCmdHandle subscribe_rtl_cmd(const RtlCmdCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_rtl_cmd
     */
    void unsubscribe_rtl_cmd(RtlCmdHandle handle);

    /**
     * @brief Poll for 'int32_t' (blocking).
     *
     * @return One int32_t update.
     */
    int32_t rtl_cmd() const;

    /**
     * @brief Callback type for subscribe_kill_cmd.
     */
    using KillCmdCallback = std::function<void(int32_t)>;

    /**
     * @brief Handle type for subscribe_kill_cmd.
     */
    using KillCmdHandle = Handle<int32_t>;

    /**
     * @brief
     */
    KillCmdHandle subscribe_kill_cmd(const KillCmdCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_kill_cmd
     */
    void unsubscribe_kill_cmd(KillCmdHandle handle);

    /**
     * @brief Poll for 'int32_t' (blocking).
     *
     * @return One int32_t update.
     */
    int32_t kill_cmd() const;

    /**
     * @brief Callback type for subscribe_color_led_cmd.
     */
    using ColorLedCmdCallback = std::function<void(LedInfo)>;

    /**
     * @brief Handle type for subscribe_color_led_cmd.
     */
    using ColorLedCmdHandle = Handle<LedInfo>;

    /**
     * @brief
     */
    ColorLedCmdHandle subscribe_color_led_cmd(const ColorLedCmdCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_color_led_cmd
     */
    void unsubscribe_color_led_cmd(ColorLedCmdHandle handle);

    /**
     * @brief Poll for 'LedInfo' (blocking).
     *
     * @return One LedInfo update.
     */
    LedInfo color_led_cmd() const;

    /**
     * @brief Copy constructor.
     */
    LumosServer(const LumosServer& other);

    /**
     * @brief Equality operator (object is not copyable).
     */
    const LumosServer& operator=(const LumosServer&) = delete;

private:
    /** @private Underlying implementation, set at instantiation */
    std::unique_ptr<LumosServerImpl> _impl;
};

} // namespace mavsdk