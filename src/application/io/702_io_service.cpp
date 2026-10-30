#include "702_io_service.h"

#include <ETH.h>
#include "esp_timer.h"
#include "702_signal_map.h"

namespace ind::application::io {

namespace {
uint32_t NowMs() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

template<class T> void Store(IoValue<T>& pump_signal, IoResult pump_result, T pump_value, uint32_t pump_stamp) {
    pump_signal.attempt_at = pump_stamp;
    pump_signal.result = pump_result;
    pump_signal.valid = pump_result == IoResult::Ok;
    if (pump_signal.valid) {
        pump_signal.value = pump_value;
        pump_signal.at_ms = pump_stamp;
    }
}
template<class T> void Expire(IoValue<T>& pump_signal, uint32_t sample_ms, uint32_t now_ms) {
    pump_signal.valid = pump_signal.valid && static_cast<uint32_t>(now_ms - pump_signal.at_ms) <= sample_ms;
}
uint32_t Older(uint32_t first_at, uint32_t second_at, uint32_t now_ms) {
    return now_ms - first_at > now_ms - second_at ? first_at : second_at;
}
control::Drive StopDrive(control::Drive pump_drive) {
    pump_drive.enabled = false;
    pump_drive.bucket_dir = 0;
    pump_drive.steer_dir = 0;
    pump_drive.bucket_effort = 0;
    pump_drive.steer_effort = 0;
    return pump_drive;
}
}

PumpIo& Io702::Cache(control::Side pump_side) {
    return pump_side == control::Side::Left ? io_status_.left_pump : io_status_.right_pump;
}

const PumpConfig& Io702::Config(control::Side pump_side) const {
    return pump_side == control::Side::Left ? pump_config_.left_pump : pump_config_.right_pump;
}

bool Io702::Start(const NetworkConfig& m31_config, const IoConfig& pump_config, const IoHooks& pump_hooks) {
    if (io_task_ != nullptr || pump_config.sample_ms == 0 || pump_config.sample_ms > 60000 ||
        pump_config.retry_ms == 0 || pump_config.retry_ms > 60000 || pump_config.refresh_ms == 0 || pump_config.refresh_ms > 60000) return false;
    for (const PumpConfig* side_config : {&pump_config.left_pump, &pump_config.right_pump}) {
        if ((side_config->bucket_position.position_ready && !ValidPosition(side_config->bucket_position)) ||
            (side_config->steer_position.position_ready && !ValidPosition(side_config->steer_position)) ||
            (side_config->valve_position.position_ready && !ValidPosition(side_config->valve_position)) ||
            (side_config->pump_output.output_ready && (!side_config->valve_position.position_ready || !ValidOutput(side_config->pump_output)))) return false;
    }
    m31_config_ = m31_config;
    pump_config_ = pump_config;
    pump_hooks_ = pump_hooks;
    // 先创建等待中的 M31 任务，PWM 任务启动失败时尚未构造网络对象。
    if (xTaskCreate(Run, "pump_io", 6144, this, 4, &io_task_) != pdPASS) return false;
    if (!steer_pwm_.Start(pump_config)) {
        vTaskDelete(io_task_);
        io_task_ = nullptr;
        return false;
    }
    xTaskNotifyGive(io_task_);
    return true;
}

control::Drive Io702::Request(control::Side pump_side) const {
    portENTER_CRITICAL(&io_lock_);
    const auto pump_drive = pump_side == control::Side::Left ? left_drive_ : right_drive_;
    portEXIT_CRITICAL(&io_lock_);
    return pump_drive;
}

bool Io702::set_702_io(control::Side pump_side, const control::Drive& pump_drive) {
    if (pump_side != control::Side::Left && pump_side != control::Side::Right) return false;
    if (io_task_ == nullptr) return false;
    const auto& pump_config = Config(pump_side);
    OutputPlan pump_plan;
    // 未配置输出时允许记录停止请求，但不会操作 GPIO 或 DO。
    if (pump_config.pump_output.output_ready) {
        if (!PlanOutput(pump_config.pump_output, pump_drive, NowMs(), &pump_plan)) return false;
    } else if (pump_drive.enabled || pump_drive.bucket_dir != 0 || pump_drive.steer_dir != 0) return false;
    portENTER_CRITICAL(&io_lock_);
    if (pump_side == control::Side::Left) left_drive_ = pump_drive;
    else right_drive_ = pump_drive;
    const bool write_fault = Cache(pump_side).pump_write.write_fault;
    const bool network_ready = io_status_.network_ready;
    portEXIT_CRITICAL(&io_lock_);
    const auto steer_status = steer_pwm_.Status(pump_side);
    const bool pump_fault = write_fault || steer_status.pwm_fault;
    steer_pwm_.Request(pump_side, (pump_fault || !network_ready) ? StopDrive(pump_drive) : pump_drive);
    return !pump_fault;
}

bool Io702::Apply(control::Side pump_side, const control::Drive& pump_drive) {
    return set_702_io(pump_side, pump_drive);
}

void Io702::Convert(PumpIo& pump_status, const PumpConfig& pump_config, uint32_t now_ms) const {
    Expire(pump_status.valve_current, pump_config_.sample_ms, now_ms);
    Expire(pump_status.bucket_current, pump_config_.sample_ms, now_ms);
    Expire(pump_status.steer_current, pump_config_.sample_ms, now_ms);
    Expire(pump_status.pressure_current, pump_config_.sample_ms, now_ms);
    Expire(pump_status.filter_alarm, pump_config_.sample_ms, now_ms);
    Expire(pump_status.bucket_up, pump_config_.sample_ms, now_ms);
    Expire(pump_status.bucket_down, pump_config_.sample_ms, now_ms);
    int16_t valve_position = 0;
    int32_t valve_angle = 0;
    int16_t bucket_position = 0;
    int16_t steer_position = 0;
    int32_t bucket_angle = 0;
    int32_t steer_angle = 0;
    uint32_t hydraulic_pressure = 0;
    const bool valve_valid = pump_status.valve_current.valid && ConvertPosition(pump_config.valve_position, pump_status.valve_current.value, &valve_position, &valve_angle);
    const bool bucket_valid = pump_status.bucket_current.valid && ConvertPosition(pump_config.bucket_position, pump_status.bucket_current.value, &bucket_position, &bucket_angle);
    const bool steer_valid = pump_status.steer_current.valid && ConvertPosition(pump_config.steer_position, pump_status.steer_current.value, &steer_position, &steer_angle);
    const bool pressure_valid = pump_status.pressure_current.valid && ConvertPressure(pump_config.hydraulic_pressure, pump_status.pressure_current.value, &hydraulic_pressure);
    pump_status.valve_position = {valve_position, pump_status.valve_current.at_ms, valve_valid};
    pump_status.bucket_position = {bucket_position, pump_status.bucket_current.at_ms, bucket_valid};
    pump_status.steer_position = {steer_position, pump_status.steer_current.at_ms, steer_valid};
    pump_status.bucket_angle = {bucket_angle, pump_status.bucket_current.at_ms, bucket_valid && pump_config.bucket_position.angle_ready};
    pump_status.steer_angle = {steer_angle, pump_status.steer_current.at_ms, steer_valid && pump_config.steer_position.angle_ready};
    pump_status.hydraulic_pressure = {hydraulic_pressure, pump_status.pressure_current.at_ms, pressure_valid};
    if (now_ms - pump_status.limits_at > pump_config_.sample_ms) {
        pump_status.bucket_limits.valid = false;
        pump_status.steer_limits.valid = false;
    }
}

IoStatus Io702::get_702_io() const {
    portENTER_CRITICAL(&io_lock_);
    IoStatus pump_status = io_status_;
    portEXIT_CRITICAL(&io_lock_);
    const uint32_t now_ms = NowMs();
    Expire(pump_status.main_oil_low, pump_config_.sample_ms, now_ms);
    Expire(pump_status.main_oil_hot, pump_config_.sample_ms, now_ms);
    Convert(pump_status.left_pump, pump_config_.left_pump, now_ms);
    Convert(pump_status.right_pump, pump_config_.right_pump, now_ms);
    pump_status.left_pump.steer_pwm = steer_pwm_.Status(control::Side::Left);
    pump_status.right_pump.steer_pwm = steer_pwm_.Status(control::Side::Right);
    return pump_status;
}

control::ReadResult Io702::Read(control::Side pump_side, control::IoSample* pump_sample, uint32_t* sample_at) {
    if (pump_sample == nullptr || sample_at == nullptr) return control::ReadResult::Fault;
    const auto io_status = get_702_io();
    const auto& pump_status = pump_side == control::Side::Left ? io_status.left_pump : io_status.right_pump;
    const auto& pump_config = Config(pump_side);
    // 未标定、未采集完成时不构造假的零位置，也不把缺配置当作传感器损坏。
    if (!pump_config.bucket_position.position_ready || !pump_config.steer_position.position_ready ||
        (pump_config.pump_output.output_ready && !pump_config.valve_position.position_ready) ||
        (pump_config_.alarm_interlock && !pump_config_.alarms_ready)) return control::ReadResult::Empty;
    const bool position_seen = pump_status.bucket_current.attempt_at != 0 && pump_status.steer_current.attempt_at != 0 &&
        (!pump_config.pump_output.output_ready || pump_status.valve_current.attempt_at != 0);
    const bool position_valid = pump_status.bucket_position.valid && pump_status.steer_position.valid &&
        (!pump_config.pump_output.output_ready || pump_status.valve_position.valid);
    const bool alarm_seen = io_status.main_oil_low.attempt_at != 0 && io_status.main_oil_hot.attempt_at != 0 && pump_status.filter_alarm.attempt_at != 0;
    const bool alarm_valid = io_status.main_oil_low.valid && io_status.main_oil_hot.valid && pump_status.filter_alarm.valid;
    if ((position_seen && !position_valid) || (pump_config_.alarm_interlock && alarm_seen && !alarm_valid)) {
        *pump_sample = {};
        pump_sample->input_fault = true;
        *sample_at = NowMs();
        return control::ReadResult::Sample;
    }
    if (!io_status.network_ready || !position_valid || (pump_config_.alarm_interlock && !alarm_valid)) return control::ReadResult::Empty;
    control::IoSample pump_next;
    pump_next.valid = true;
    pump_next.bucket_pct = pump_status.bucket_position.value;
    pump_next.steer_pct = pump_status.steer_position.value;
    pump_next.valve_pct = pump_status.valve_position.value;
    pump_next.valve_valid = pump_status.valve_position.valid;
    pump_next.bucket_limits = pump_status.bucket_limits;
    pump_next.steer_limits = pump_status.steer_limits;
    pump_next.mechanism_fault = pump_config_.alarm_interlock && (io_status.main_oil_low.value == pump_config_.oil_low_active ||
        io_status.main_oil_hot.value == pump_config_.oil_hot_active || pump_status.filter_alarm.value == pump_config_.filter_active);
    const uint32_t now_ms = NowMs();
    uint32_t pump_stamp = Older(pump_status.bucket_current.at_ms, pump_status.steer_current.at_ms, now_ms);
    if (pump_config.pump_output.output_ready) pump_stamp = Older(pump_stamp, pump_status.valve_current.at_ms, now_ms);
    if (pump_config_.alarm_interlock) {
        pump_stamp = Older(pump_stamp, io_status.main_oil_low.at_ms, now_ms);
        pump_stamp = Older(pump_stamp, io_status.main_oil_hot.at_ms, now_ms);
        pump_stamp = Older(pump_stamp, pump_status.filter_alarm.at_ms, now_ms);
    }
    *sample_at = pump_stamp;
    *pump_sample = pump_next;
    return control::ReadResult::Sample;
}

bool Io702::ReadOutput(control::Side pump_side, control::OutputSample* pump_feedback) {
    if (pump_feedback == nullptr) return false;
    portENTER_CRITICAL(&io_lock_);
    *pump_feedback = Cache(pump_side).pump_feedback;
    portEXIT_CRITICAL(&io_lock_);
    return true;
}

uint32_t Io702::Recovered(control::Side pump_side, uint32_t pump_faults) {
    const bool pwm_fault = steer_pwm_.Status(pump_side).pwm_fault;
    portENTER_CRITICAL(&io_lock_);
    auto& pump_status = Cache(pump_side);
    const auto& pump_feedback = pump_status.pump_feedback;
    const bool pump_safe = pump_feedback.bucket.valid && pump_feedback.steer.valid &&
        pump_feedback.bucket.applied.direction == 0 && pump_feedback.steer.applied.direction == 0 &&
        NowMs() - pump_feedback.bucket.at_ms <= pump_config_.sample_ms && NowMs() - pump_feedback.steer.at_ms <= pump_config_.sample_ms;
    const uint32_t pump_recovered = pump_safe && !pwm_fault ? pump_status.recovered_faults & pump_faults : 0U;
    if ((pump_recovered & static_cast<uint32_t>(control::FaultCode::Output)) != 0) pump_status.pump_write.write_fault = false;
    portEXIT_CRITICAL(&io_lock_);
    return pump_recovered;
}

void Io702::Run(void* pump_context) {
    auto* pump_self = static_cast<Io702*>(pump_context);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const auto& m31_config = pump_self->m31_config_;
    // 构造、全部调用及生命周期都在这个任务，同事底层保持原样。
    devices::m31remoteio::M31RemoteIO m31_device(m31_config.m31_address, m31_config.m31_port, m31_config.m31_unit, m31_config.connect_ms, m31_config.response_ms);
    uint8_t channel_index = 0;
    for (;;) {
        const bool network_ready = ETH.linkUp() && ETH.hasIP();
        portENTER_CRITICAL(&pump_self->io_lock_);
        pump_self->io_status_.network_ready = network_ready;
        portEXIT_CRITICAL(&pump_self->io_lock_);
        pump_self->Hooks(control::Side::Left);
        pump_self->Hooks(control::Side::Right);
        if (network_ready) {
            pump_self->Write(m31_device, control::Side::Left);
            pump_self->Write(m31_device, control::Side::Right);
            pump_self->Poll(m31_device, channel_index);
            channel_index = (channel_index + 1U) % 16U;
        } else {
            pump_self->steer_pwm_.Request(control::Side::Left, StopDrive(pump_self->Request(control::Side::Left)));
            pump_self->steer_pwm_.Request(control::Side::Right, StopDrive(pump_self->Request(control::Side::Right)));
        }
        vTaskDelay(pdMS_TO_TICKS(network_ready ? 5 : pump_self->pump_config_.retry_ms));
    }
}

void Io702::Hooks(control::Side pump_side) {
    control::AxisLimits bucket_limits;
    control::AxisLimits steer_limits;
    control::OutputSample pump_feedback;
    uint32_t limits_at = 0;
    const auto pump_request = Request(pump_side);
    if (pump_hooks_.pump_limits == nullptr || !pump_hooks_.pump_limits(pump_side, &bucket_limits, &steer_limits, &limits_at)) {
        bucket_limits.valid = false;
        steer_limits.valid = false;
    }
    if (pump_hooks_.pump_feedback == nullptr || !pump_hooks_.pump_feedback(pump_side, pump_request, &pump_feedback)) pump_feedback = {};
    const uint32_t pump_recovered = pump_hooks_.pump_recovered == nullptr ? 0U : pump_hooks_.pump_recovered(pump_side);
    portENTER_CRITICAL(&io_lock_);
    auto& pump_status = Cache(pump_side);
    pump_status.bucket_limits = bucket_limits;
    pump_status.steer_limits = steer_limits;
    pump_status.limits_at = limits_at;
    pump_status.pump_feedback = pump_feedback;
    pump_status.recovered_faults = pump_recovered;
    portEXIT_CRITICAL(&io_lock_);
}

void Io702::Poll(devices::m31remoteio::M31RemoteIO& m31_device, uint8_t channel_index) {
    const uint32_t sample_at = NowMs();
    bool digital_value = false;
    int32_t analog_value = 0;
    IoResult io_result = IoResult::InvalidChannel;
    IoValue<bool>* digital_target = nullptr;
    IoValue<int32_t>* analog_target = nullptr;
    // 每轮只采一个通道，输出停止请求可在相邻网络事务之间优先处理。
    switch (channel_index) {
    case 0: digital_target = &io_status_.main_oil_low; io_result = m31_device.get_di(map::di::main_oil_low, digital_value); break;
    case 1: digital_target = &io_status_.main_oil_hot; io_result = m31_device.get_di(map::di::main_oil_hot, digital_value); break;
    case 2: digital_target = &io_status_.left_pump.filter_alarm; io_result = m31_device.get_di(map::di::left_filter_alarm, digital_value); break;
    case 3: digital_target = &io_status_.right_pump.filter_alarm; io_result = m31_device.get_di(map::di::right_filter_alarm, digital_value); break;
    case 4: analog_target = &io_status_.left_pump.valve_current; io_result = m31_device.get_ai_current(map::ai::left_valve_pos, analog_value); break;
    case 5: analog_target = &io_status_.left_pump.bucket_current; io_result = m31_device.get_ai_current(map::ai::left_bucket_pos, analog_value); break;
    case 6: analog_target = &io_status_.left_pump.steer_current; io_result = m31_device.get_ai_current(map::ai::left_steer_pos, analog_value); break;
    case 7: analog_target = &io_status_.left_pump.pressure_current; io_result = m31_device.get_ai_current(map::ai::left_pressure, analog_value); break;
    case 8: analog_target = &io_status_.right_pump.valve_current; io_result = m31_device.get_ai_current(map::ai::right_valve_pos, analog_value); break;
    case 9: analog_target = &io_status_.right_pump.bucket_current; io_result = m31_device.get_ai_current(map::ai::right_bucket_pos, analog_value); break;
    case 10: analog_target = &io_status_.right_pump.steer_current; io_result = m31_device.get_ai_current(map::ai::right_steer_pos, analog_value); break;
    case 11: analog_target = &io_status_.right_pump.pressure_current; io_result = m31_device.get_ai_current(map::ai::right_pressure, analog_value); break;
    case 12: digital_target = &io_status_.left_pump.bucket_up; io_result = m31_device.get_do(map::dout::left_bucket_up, digital_value); break;
    case 13: digital_target = &io_status_.left_pump.bucket_down; io_result = m31_device.get_do(map::dout::left_bucket_down, digital_value); break;
    case 14: digital_target = &io_status_.right_pump.bucket_up; io_result = m31_device.get_do(map::dout::right_bucket_up, digital_value); break;
    case 15: digital_target = &io_status_.right_pump.bucket_down; io_result = m31_device.get_do(map::dout::right_bucket_down, digital_value); break;
    default: return;
    }
    portENTER_CRITICAL(&io_lock_);
    if (digital_target != nullptr) Store(*digital_target, io_result, digital_value, sample_at);
    if (analog_target != nullptr) Store(*analog_target, io_result, analog_value, sample_at);
    portEXIT_CRITICAL(&io_lock_);
    if (io_result != IoResult::Ok) vTaskDelay(pdMS_TO_TICKS(pump_config_.retry_ms));
}

void Io702::Write(devices::m31remoteio::M31RemoteIO& m31_device, control::Side pump_side) {
    const auto& pump_config = Config(pump_side).pump_output;
    if (!pump_config.output_ready) return;
    const auto pump_drive = Request(pump_side);
    auto& pump_work = pump_side == control::Side::Left ? left_write_ : right_write_;
    portENTER_CRITICAL(&io_lock_);
    const auto write_status = Cache(pump_side).pump_write;
    portEXIT_CRITICAL(&io_lock_);
    WriteAction output_action;
    if (!pump_work.Prepare(pump_config, pump_drive, write_status.write_fault, NowMs(), pump_config_.refresh_ms, pump_config_.retry_ms, &output_action)) return;
    const auto output_drive = pump_work.Current();
    // 与先前已完成的请求不一致时，取消旧 PWM 许可；停止请求可以立即到本地 PWM。
    if (!SameDrive(output_drive, write_status.pump_request) || !output_drive.enabled) steer_pwm_.Request(pump_side, StopDrive(output_drive));
    const uint16_t bucket_up = pump_side == control::Side::Left ? map::dout::left_bucket_up : map::dout::right_bucket_up;
    const uint16_t bucket_down = pump_side == control::Side::Left ? map::dout::left_bucket_down : map::dout::right_bucket_down;
    IoResult write_result = IoResult::InvalidValue;
    switch (output_action.output_channel) {
    case OutputChannel::BucketUp: write_result = m31_device.set_do(bucket_up, output_action.output_value != 0); break;
    case OutputChannel::BucketDown: write_result = m31_device.set_do(bucket_down, output_action.output_value != 0); break;
    }
    const uint32_t write_at = NowMs();
    pump_work.Complete(write_result == IoResult::Ok, write_at);
    portENTER_CRITICAL(&io_lock_);
    auto& pump_status = Cache(pump_side).pump_write;
    pump_status.pump_request = output_drive;
    pump_status.write_result = write_result;
    pump_status.write_at = write_at;
    pump_status.write_complete = pump_work.Finished();
    pump_status.write_fault = pump_status.write_fault || write_result != IoResult::Ok;
    pump_status.write_uncertain = pump_work.Uncertain();
    portEXIT_CRITICAL(&io_lock_);
    if (write_result != IoResult::Ok) {
        steer_pwm_.Request(pump_side, StopDrive(output_drive));
        return;
    }
    if (pump_work.Finished() && !write_status.write_fault) {
        const auto pump_latest = Request(pump_side);
        if (SameDrive(pump_latest, output_drive) && DriveFresh(pump_latest, write_at)) {
            steer_pwm_.Confirm(pump_side, pump_latest);
            steer_pwm_.Request(pump_side, pump_latest);
        }
    }
}

} // namespace ind::application::io
