#pragma once

#include "can/702_can_protocol.h"
#include "control/pump_link.h"
#include "io/702_io_service.h"

namespace ind::application {

struct AlarmRoutes {
    bool enabled = false;
    can::AlarmRoute oil_low;
    can::AlarmRoute oil_hot;
    can::AlarmRoute left_filter;
    can::AlarmRoute right_filter;
};

struct AppConfig {
    io::NetworkConfig m31_network;
    io::IoConfig pump_io;
    io::IoHooks pump_hooks;
    control::Settings left_control;
    control::Settings right_control;
    can::ProtocolSettings pump_protocol;
    AlarmRoutes alarm_routes;
};

struct CanStatus {
    bool can_ready = false;
    bool bus_ready = false;
    twai_error_state_t bus_state = TWAI_ERROR_BUS_OFF;
    esp_err_t bus_result = ESP_ERR_INVALID_STATE;
    esp_err_t statistics_result = ESP_ERR_INVALID_STATE;
    Esp32TwaiStatistics bus_statistics{};
    control::Counters link_statistics{};
    control::Snapshot left_pump;
    control::Snapshot right_pump;
};

// 开机配置只在此处集中定义；通道号仍只在 702_signal_map.h 修改。
AppConfig Make702Config();

class App702 final : private control::StatusCodec {
public:
    static App702& Instance();
    esp_err_t Start(const AppConfig& pump_config);
    CanStatus get_702_can() const;
    io::IoStatus get_702_io() const;
    // 其它设备模块补充真实测量；at_ms 必须使用同一 esp_timer 毫秒时钟。
    // 不接受把接收的控制指令回填为已执行状态。
    bool set_702_status(control::Side pump_side, const can::PumpStatus& pump_status);
    esp_err_t ResetPump(control::Side pump_side);

private:
    App702();
    control::EncodeResult Encode(const control::Snapshot& pump_status, uint8_t pump_slot, can::Frame* pump_frame) override;
    io::Io702 pump_io_;
    control::PumpLink pump_link_;
    can::ProtocolSettings pump_protocol_;
    AlarmRoutes alarm_routes_;
    mutable portMUX_TYPE status_lock_ = portMUX_INITIALIZER_UNLOCKED;
    can::PumpStatus left_status_;
    can::PumpStatus right_status_;
    bool app_started_ = false;
    bool can_ready_ = false;
};

} // namespace ind::application
