#include "702_app.h"

namespace ind::application {

namespace {
io::PositionConfig PositionPoints(int32_t position_low, int32_t position_mid, int32_t position_high) {
    io::PositionConfig position_config;
    position_config.position_ready = true;
    position_config.position_low = position_low;
    position_config.position_mid = position_mid;
    position_config.position_high = position_high;
    // 先换算为 -100/0/100；真实角度未测量时不生成角度反馈。
    return position_config;
}

control::AxisSettings AxisProfile(uint16_t axis_effort) {
    control::AxisSettings axis_config;
    // 以下仅为离线逻辑测试的暂定值，现场需分别确认四轴参数。
    axis_config.min_pct = -100;
    axis_config.max_pct = 100;
    axis_config.start_tol = 4;
    axis_config.stop_tol = 2;
    axis_config.min_progress = 1;
    axis_config.wrong_tol = 3;
    axis_config.effort = axis_effort;
    axis_config.ack_ms = 2000;
    axis_config.travel_ms = 20000;
    axis_config.stall_ms = 4000;
    axis_config.grace_ms = 1000;
    axis_config.reverse_ms = 300;
    return axis_config;
}

control::Settings PumpSettings() {
    control::Settings pump_control;
    // CAN 的 200ms/50ms 周期各容忍三个周期；现场还需确认 IO 的最坏采样延迟。
    pump_control.auth_ms = 600;
    pump_control.motion_ms = 150;
    pump_control.io_ms = 1000;
    pump_control.stop_policy = control::StopPolicy::Together;
    return pump_control;
}
}

AppConfig Make702Config() {
    AppConfig pump_config;
    pump_config.left_control = PumpSettings();
    pump_config.right_control = PumpSettings();
    pump_config.left_control.bucket = AxisProfile(1000);
    pump_config.left_control.steer = AxisProfile(250);
    pump_config.right_control.bucket = AxisProfile(1000);
    pump_config.right_control.steer = AxisProfile(250);

    // 四个传感器各自保存三点电流，单位 uA。允许递增或递减，不能重复或乱序。
    // 4/12/20mA 是可替换的暂定比例；分别实测最低/中位/最高机械位置后填写。
    pump_config.pump_io.left_pump.bucket_position = PositionPoints(4000, 12000, 20000);
    pump_config.pump_io.left_pump.steer_position = PositionPoints(4000, 12000, 20000);
    pump_config.pump_io.right_pump.bucket_position = PositionPoints(4000, 12000, 20000);
    pump_config.pump_io.right_pump.steer_position = PositionPoints(4000, 12000, 20000);

    // 阀芯 AI 的中位暂按 12mA；现场异常未排查，暂不产生有效阀芯位置。
    // 左右阀芯分别预留 AI1/AI5，需实测低、中、高三点后再启用标定。
    // 仅由 GPIO4/5 PWM 输出转舵。以下占空比是离线示例，输出保持禁用。
    pump_config.pump_io.left_pump.pump_output.steer_stop = 500;
    pump_config.pump_io.left_pump.pump_output.steer_left = 250;
    pump_config.pump_io.left_pump.pump_output.steer_right = 750;
    pump_config.pump_io.right_pump.pump_output.steer_stop = 500;
    pump_config.pump_io.right_pump.pump_output.steer_left = 250;
    pump_config.pump_io.right_pump.pump_output.steer_right = 750;

    // 油位/油温/滤堵只采集，不参与四轴联锁。位置有效性和运动监测仍参与闭环。
    pump_config.pump_io.alarm_interlock = false;
    // 液压压力先保留原始电流；传感器满量程未确认，pressure_ready 仍为 false。
    // 0x06069002/82 的压力为上报实测值，不是目标压力；不新增泵控制报文或压力调节。
    // output_ready 仍为 false，PWM 频率仍为 0；App702 在该模式下保持四轴 Disabled。
    // 部署前确认 DO 极性、PWM 频率与方向，接入限位和执行反馈依据。
    // DI 报警暂按液压设备分类；SPN/FMI 未定，不开启报警映射或报警帧。
    pump_config.alarm_routes.oil_low.device_type = 2;
    pump_config.alarm_routes.oil_hot.device_type = 2;
    pump_config.alarm_routes.left_filter.device_type = 2;
    pump_config.alarm_routes.right_filter.device_type = 2;
    pump_config.alarm_routes.oil_hot.active_level = false;
    // engine/clutch/backflush/gearbox/hydraulic_running 的真实反馈仍由 set_702_status 补充。
    return pump_config;
}

} // namespace ind::application
