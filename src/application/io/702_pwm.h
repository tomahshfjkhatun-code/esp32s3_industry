#pragma once

#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "702_io_model.h"

namespace ind::application::io {

struct PwmStatus {
    bool pwm_ready = false;
    bool pwm_fault = false;
    uint16_t steer_duty = 0;
    uint32_t written_at = 0;
};

// 本任务独占 LEDC；M31 阻塞不会阻止本地 PWM 的租约检查。
class Pwm702 final {
public:
    bool Start(const IoConfig& pump_config);
    void Request(control::Side pump_side, const control::Drive& pump_drive);
    void Confirm(control::Side pump_side, const control::Drive& pump_drive);
    PwmStatus Status(control::Side pump_side) const;

private:
    struct PwmSlot {
        control::Drive pump_request;
        control::Drive pump_confirmed;
        bool pump_granted = false;
        PwmStatus pump_status;
    };
    static void Run(void* pump_context);
    void Update(uint8_t pump_index);
    mutable portMUX_TYPE pwm_lock_ = portMUX_INITIALIZER_UNLOCKED;
    PwmSlot pump_slots_[2];
    IoConfig pump_config_;
    TaskHandle_t pwm_task_ = nullptr;
};

} // namespace ind::application::io
