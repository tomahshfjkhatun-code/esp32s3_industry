#include "702_app.h"

#include <cstdio>
#include <cstring>
#include "esp_timer.h"
#include "../shell/shell.h"

namespace ind::application {

namespace {
const char* PumpMode(control::Mode pump_mode) {
    switch (pump_mode) {
    case control::Mode::Waiting: return "waiting";
    case control::Mode::Tracking: return "tracking";
    case control::Mode::Holding: return "holding";
    case control::Mode::Fault: return "fault";
    default: return "unknown";
    }
}

int PumpStatusCommand(int pump_argc, char** pump_argv) {
    if (pump_argc != 1) return 1;
    (void)pump_argv;
    const auto pump_can = App702::Instance().get_702_can();
    const auto pump_io = App702::Instance().get_702_io();
    std::printf("702 CAN ready=%d bus=%d state=%d result=%ld ETH=%d\n", pump_can.can_ready, pump_can.bus_ready, static_cast<int>(pump_can.bus_state), static_cast<long>(pump_can.bus_result), pump_io.network_ready);
    std::printf("left mode=%s wait=%u faults=0x%lx bucket=%d/%d steer=%d/%d raw=%ld,%ld,%ld write=%d uncertain=%d\n",
        PumpMode(pump_can.left_pump.mode), static_cast<unsigned>(pump_can.left_pump.wait), static_cast<unsigned long>(pump_can.left_pump.faults),
        pump_io.left_pump.bucket_position.value, pump_io.left_pump.bucket_position.valid, pump_io.left_pump.steer_position.value, pump_io.left_pump.steer_position.valid,
        static_cast<long>(pump_io.left_pump.bucket_current.value), static_cast<long>(pump_io.left_pump.steer_current.value), static_cast<long>(pump_io.left_pump.pressure_current.value),
        static_cast<int>(pump_io.left_pump.pump_write.write_result), pump_io.left_pump.pump_write.write_uncertain);
    std::printf("right mode=%s wait=%u faults=0x%lx bucket=%d/%d steer=%d/%d raw=%ld,%ld,%ld write=%d uncertain=%d\n",
        PumpMode(pump_can.right_pump.mode), static_cast<unsigned>(pump_can.right_pump.wait), static_cast<unsigned long>(pump_can.right_pump.faults),
        pump_io.right_pump.bucket_position.value, pump_io.right_pump.bucket_position.valid, pump_io.right_pump.steer_position.value, pump_io.right_pump.steer_position.valid,
        static_cast<long>(pump_io.right_pump.bucket_current.value), static_cast<long>(pump_io.right_pump.steer_current.value), static_cast<long>(pump_io.right_pump.pressure_current.value),
        static_cast<int>(pump_io.right_pump.pump_write.write_result), pump_io.right_pump.pump_write.write_uncertain);
    std::printf("valve AI left=%ld/%d pct=%d/%d right=%ld/%d pct=%d/%d\n", static_cast<long>(pump_io.left_pump.valve_current.value), pump_io.left_pump.valve_current.valid, pump_io.left_pump.valve_position.value, pump_io.left_pump.valve_position.valid, static_cast<long>(pump_io.right_pump.valve_current.value), pump_io.right_pump.valve_current.valid, pump_io.right_pump.valve_position.value, pump_io.right_pump.valve_position.valid);
    std::printf("TX queued=%lu sent=%llu failed=%llu timeout=%llu missing=%lu encode=%lu queue_fail=%lu\n",
        static_cast<unsigned long>(pump_can.link_statistics.queued), static_cast<unsigned long long>(pump_can.bus_statistics.sent),
        static_cast<unsigned long long>(pump_can.bus_statistics.terminal_failed), static_cast<unsigned long long>(pump_can.bus_statistics.tx_timeout),
        static_cast<unsigned long>(pump_can.link_statistics.not_ready), static_cast<unsigned long>(pump_can.link_statistics.encode_failed),
        static_cast<unsigned long>(pump_can.link_statistics.enqueue_failed));
    std::printf("reset accepted=%lu rejected=%lu RX overflow=%lu\n",
        static_cast<unsigned long>(pump_can.link_statistics.reset_done), static_cast<unsigned long>(pump_can.link_statistics.reset_rejected),
        static_cast<unsigned long>(pump_can.link_statistics.rx_dropped));
    return 0;
}

int PumpResetCommand(int pump_argc, char** pump_argv) {
    if (pump_argc != 2) return 1;
    control::Side pump_side;
    if (std::strcmp(pump_argv[1], "left") == 0) pump_side = control::Side::Left;
    else if (std::strcmp(pump_argv[1], "right") == 0) pump_side = control::Side::Right;
    else return 1;
    const esp_err_t pump_result = App702::Instance().ResetPump(pump_side);
    std::printf("reset queued: %s; see 702_status for acceptance counters\n", esp_err_to_name(pump_result));
    return pump_result == ESP_OK ? 0 : 1;
}

IndShellCommand pump_status_command{"702_status", "Read 702 CAN and IO snapshots", nullptr, PumpStatusCommand, nullptr};
IndShellCommand pump_reset_command{"702_reset", "Queue reset; requires recovery evidence and new CAN authority", "<left|right>", PumpResetCommand, nullptr};
}

App702::App702() : pump_link_(pump_io_, *this) {}

App702& App702::Instance() {
    static App702 pump_app;
    return pump_app;
}

esp_err_t App702::Start(const AppConfig& pump_config) {
    if (app_started_) return ESP_ERR_INVALID_STATE;
    // 先校验纯状态机配置，避免配置错误后留下已启动的 IO 任务。
    control::PumpControl left_probe(control::Side::Left);
    control::PumpControl right_probe(control::Side::Right);
    if (!left_probe.Configure(pump_config.left_control) || !right_probe.Configure(pump_config.right_control) ||
        pump_config.left_control.stop_policy != pump_config.right_control.stop_policy) return ESP_ERR_INVALID_ARG;
    auto left_control = pump_config.left_control;
    auto right_control = pump_config.right_control;
    // 关闭硬件输出时也禁用轴调度，避免采集模式等待不存在的输出确认而报超时。
    if (!pump_config.pump_io.left_pump.pump_output.output_ready) {
        left_control.bucket = {};
        left_control.steer = {};
    }
    if (!pump_config.pump_io.right_pump.pump_output.output_ready) {
        right_control.bucket = {};
        right_control.steer = {};
    }
    if (pump_config.pump_protocol.status_ms == 0 || pump_config.pump_protocol.status_ms > 60000) return ESP_ERR_INVALID_ARG;
    const auto& routes = pump_config.alarm_routes;
    if (routes.enabled && (!can::ValidRoute(routes.oil_low) || !can::ValidRoute(routes.oil_hot) ||
        !can::ValidRoute(routes.left_filter) || !can::ValidRoute(routes.right_filter))) return ESP_ERR_INVALID_ARG;
    pump_protocol_ = pump_config.pump_protocol;
    alarm_routes_ = routes;
    if (!pump_io_.Start(pump_config.m31_network, pump_config.pump_io, pump_config.pump_hooks)) return ESP_ERR_INVALID_ARG;
    app_started_ = true;
    const esp_err_t pump_result = pump_link_.Start(left_control, right_control);
    can_ready_ = pump_result == ESP_OK;
    // 必须在同事 Shell::Initialize 之前注册；现有命令和注册机制保持不变。
    ind_shell_register_command(&pump_status_command);
    ind_shell_register_command(&pump_reset_command);
    return pump_result;
}

CanStatus App702::get_702_can() const {
    CanStatus pump_status;
    pump_status.can_ready = can_ready_;
    pump_status.bus_ready = pump_link_.BusReady();
    pump_status.bus_result = can::CanService::Instance().State(&pump_status.bus_state);
    pump_status.statistics_result = can::CanService::Instance().Statistics(&pump_status.bus_statistics);
    pump_status.link_statistics = pump_link_.Statistics();
    pump_status.left_pump = pump_link_.Status(control::Side::Left);
    pump_status.right_pump = pump_link_.Status(control::Side::Right);
    return pump_status;
}

io::IoStatus App702::get_702_io() const { return pump_io_.get_702_io(); }

bool App702::set_702_status(control::Side pump_side, const can::PumpStatus& pump_status) {
    if (pump_side != control::Side::Left && pump_side != control::Side::Right) return false;
    portENTER_CRITICAL(&status_lock_);
    if (pump_side == control::Side::Left) left_status_ = pump_status;
    else right_status_ = pump_status;
    portEXIT_CRITICAL(&status_lock_);
    return true;
}

esp_err_t App702::ResetPump(control::Side pump_side) { return pump_link_.RequestReset(pump_side); }

control::EncodeResult App702::Encode(const control::Snapshot& pump_status, uint8_t pump_slot, can::Frame* pump_frame) {
    if (pump_frame == nullptr) return control::EncodeResult::Invalid;
    portENTER_CRITICAL(&status_lock_);
    can::PumpStatus status_values = pump_status.side == control::Side::Left ? left_status_ : right_status_;
    portEXIT_CRITICAL(&status_lock_);
    const auto io_status = pump_io_.get_702_io();
    const auto& side_status = pump_status.side == control::Side::Left ? io_status.left_pump : io_status.right_pump;
    status_values.bucket_angle = side_status.bucket_angle;
    status_values.steer_angle = side_status.steer_angle;
    status_values.hydraulic_pressure = side_status.hydraulic_pressure;
    status_values.valve_position = side_status.valve_position;
    if (pump_slot == 3 || pump_slot == 4) {
        if (!alarm_routes_.enabled) return control::EncodeResult::NotReady;
        else {
            const can::Signal<bool> oil_low{io_status.main_oil_low.value, io_status.main_oil_low.at_ms, io_status.main_oil_low.valid};
            const can::Signal<bool> oil_hot{io_status.main_oil_hot.value, io_status.main_oil_hot.at_ms, io_status.main_oil_hot.valid};
            const can::Signal<bool> filter{side_status.filter_alarm.value, side_status.filter_alarm.at_ms, side_status.filter_alarm.valid};
            const auto& filter_route = pump_status.side == control::Side::Left ? alarm_routes_.left_filter : alarm_routes_.right_filter;
            const bool first = pump_slot == 3 ? can::MapAlarm(alarm_routes_.oil_low, oil_low, &status_values.pump_alarms[0]) :
                can::MapAlarm(filter_route, filter, &status_values.pump_alarms[0]);
            const bool second = can::MapAlarm(pump_slot == 3 ? alarm_routes_.oil_hot : filter_route, pump_slot == 3 ? oil_hot : filter, &status_values.pump_alarms[1]);
            if (!first || !second) return control::EncodeResult::NotReady;
        }
    }
    // 内部故障位不对应协议 SPN/FMI；报警编码只接受已配置的映射和新鲜 DI。
    can::StatusFrame status_frame;
    const auto pump_result = can::EncodeStatus(pump_status.side == control::Side::Right, pump_slot == 4 ? 3 : pump_slot, status_values, pump_protocol_, static_cast<uint32_t>(esp_timer_get_time() / 1000), &status_frame);
    if (pump_result != can::EncodeResult::Ready) return pump_result;
    pump_frame->id = status_frame.id;
    pump_frame->dlc = status_frame.dlc;
    for (uint8_t pump_index = 0; pump_index < 8; ++pump_index) pump_frame->data[pump_index] = status_frame.data[pump_index];
    return pump_result;
}

} // namespace ind::application
