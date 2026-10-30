#include "702_pwm.h"

#include <Arduino.h>
#include "esp_timer.h"
#include "702_signal_map.h"

namespace ind::application::io {

namespace {
uint32_t NowMs() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }
uint8_t PumpIndex(control::Side pump_side) { return pump_side == control::Side::Left ? 0 : 1; }
uint8_t SteerPin(uint8_t pump_index) { return pump_index == 0 ? map::pwm::left_steer_gpio : map::pwm::right_steer_gpio; }
}

bool Pwm702::Start(const IoConfig& pump_config) {
    if (pwm_task_ != nullptr || map::pwm::left_steer_gpio == map::pwm::right_steer_gpio) return false;
    pump_config_ = pump_config;
    return xTaskCreate(Run, "pump_pwm", 3072, this, 5, &pwm_task_) == pdPASS;
}

void Pwm702::Request(control::Side pump_side, const control::Drive& pump_drive) {
    portENTER_CRITICAL(&pwm_lock_);
    pump_slots_[PumpIndex(pump_side)].pump_request = pump_drive;
    portEXIT_CRITICAL(&pwm_lock_);
}

void Pwm702::Confirm(control::Side pump_side, const control::Drive& pump_drive) {
    portENTER_CRITICAL(&pwm_lock_);
    auto& pump_slot = pump_slots_[PumpIndex(pump_side)];
    pump_slot.pump_confirmed = pump_drive;
    pump_slot.pump_granted = true;
    portEXIT_CRITICAL(&pwm_lock_);
}

PwmStatus Pwm702::Status(control::Side pump_side) const {
    portENTER_CRITICAL(&pwm_lock_);
    const auto pump_status = pump_slots_[PumpIndex(pump_side)].pump_status;
    portEXIT_CRITICAL(&pwm_lock_);
    return pump_status;
}

void Pwm702::Run(void* pump_context) {
    auto* pump_self = static_cast<Pwm702*>(pump_context);
    for (uint8_t pump_index = 0; pump_index < 2; ++pump_index) {
        const auto& pump_config = pump_index == 0 ? pump_self->pump_config_.left_pump : pump_self->pump_config_.right_pump;
        if (!pump_config.pump_output.output_ready) continue;
        const auto& steer_config = pump_config.pump_output;
        PwmStatus pump_status;
        pump_status.pwm_ready = ledcAttach(SteerPin(pump_index), steer_config.steer_frequency, steer_config.steer_resolution);
        pump_status.pwm_fault = !pump_status.pwm_ready;
        const uint32_t steer_max = (1U << steer_config.steer_resolution) - 1U;
        if (pump_status.pwm_ready) {
            pump_status.pwm_fault = !ledcWrite(SteerPin(pump_index), steer_config.steer_stop * steer_max / 1000U);
            pump_status.steer_duty = steer_config.steer_stop;
            pump_status.written_at = NowMs();
        }
        portENTER_CRITICAL(&pump_self->pwm_lock_);
        pump_self->pump_slots_[pump_index].pump_status = pump_status;
        portEXIT_CRITICAL(&pump_self->pwm_lock_);
    }
    for (;;) {
        pump_self->Update(0);
        pump_self->Update(1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void Pwm702::Update(uint8_t pump_index) {
    const auto& pump_config = pump_index == 0 ? pump_config_.left_pump : pump_config_.right_pump;
    portENTER_CRITICAL(&pwm_lock_);
    const PwmSlot pump_slot = pump_slots_[pump_index];
    portEXIT_CRITICAL(&pwm_lock_);
    if (!pump_slot.pump_status.pwm_ready) return;
    control::Drive pump_drive = pump_slot.pump_request;
    if (!pump_slot.pump_granted || !SameDrive(pump_drive, pump_slot.pump_confirmed) || pump_slot.pump_status.pwm_fault) pump_drive.enabled = false;
    OutputPlan pump_plan;
    if (!PlanOutput(pump_config.pump_output, pump_drive, NowMs(), &pump_plan)) {
        pump_plan.steer_duty = pump_config.pump_output.steer_stop;
    }
    if (pump_plan.steer_duty == pump_slot.pump_status.steer_duty && !pump_slot.pump_status.pwm_fault) return;
    const uint32_t steer_max = (1U << pump_config.pump_output.steer_resolution) - 1U;
    const bool steer_written = ledcWrite(SteerPin(pump_index), pump_plan.steer_duty * steer_max / 1000U);
    portENTER_CRITICAL(&pwm_lock_);
    auto& pump_status = pump_slots_[pump_index].pump_status;
    pump_status.pwm_fault = pump_status.pwm_fault || !steer_written;
    if (steer_written) {
        pump_status.steer_duty = pump_plan.steer_duty;
        pump_status.written_at = NowMs();
    }
    portEXIT_CRITICAL(&pwm_lock_);
}

} // namespace ind::application::io
