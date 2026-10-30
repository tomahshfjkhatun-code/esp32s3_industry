#pragma once

#include <stdint.h>
#include "../can/702_can_protocol.h"
#include "../control/pump_control.h"

namespace ind::application::io {

struct PositionConfig {
    bool position_ready = false;
    int32_t position_low = 4000;
    int32_t position_mid = 12000;
    int32_t position_high = 20000;
    bool angle_ready = false;
    int32_t angle_low = 0;
    int32_t angle_mid = 0;
    int32_t angle_high = 0;
};

struct PressureConfig {
    bool pressure_ready = false;
    int32_t pressure_low = 4000;
    int32_t pressure_high = 20000;
    uint32_t pressure_min = 0;
    uint32_t pressure_max = 0;
};

// Effort 按驱动力插值；Fixed 使用该方向的 PWM 固定端点。
enum class OutputScale : uint8_t { Effort, Fixed };

struct OutputConfig {
    bool output_ready = false;
    bool steer_profile_ready = false;
    bool bucket_active = true;
    bool bucket_reverse = false;
    uint32_t steer_frequency = 0;
    uint8_t steer_resolution = 10;
    uint16_t steer_stop = 0;
    uint16_t steer_left = 0;
    uint16_t steer_right = 0;
    OutputScale steer_pwm_scale = OutputScale::Effort;
};

struct PumpConfig {
    PositionConfig bucket_position;
    PositionConfig steer_position;
    PositionConfig valve_position;
    PressureConfig hydraulic_pressure;
    OutputConfig pump_output;
};

struct IoConfig {
    PumpConfig left_pump;
    PumpConfig right_pump;
    uint32_t sample_ms = 1000;
    uint32_t retry_ms = 500;
    uint32_t refresh_ms = 500;
    // 缺省只采集；只有明确接入报警联锁时才要求报警极性和有效采样。
    bool alarm_interlock = false;
    bool alarms_ready = false;
    bool oil_low_active = true;
    bool oil_hot_active = true;
    bool filter_active = true;
};

struct OutputPlan {
    bool bucket_up = false;
    bool bucket_down = false;
    uint16_t steer_duty = 0;
    bool pump_motion = false;
};

enum class OutputChannel : uint8_t { BucketUp, BucketDown };

struct WriteAction {
    OutputChannel output_channel = OutputChannel::BucketUp;
    int32_t output_value = 0;
};

// 纯 DO 输出顺序状态机，先断开两路，再允许 PWM；不持有网络对象。
class OutputSequence final {
public:
    bool Prepare(const OutputConfig& pump_config, const control::Drive& pump_drive, bool output_fault, uint32_t now_ms, uint32_t refresh_ms, uint32_t retry_ms, WriteAction* output_action);
    void Complete(bool output_success, uint32_t now_ms);
    control::Drive Current() const;
    bool Finished() const;
    bool Uncertain() const;

private:
    control::Drive output_drive_;
    uint8_t output_phase_ = 0;
    bool output_seen_ = false;
    bool output_failed_ = false;
    bool output_pending_ = false;
    bool output_complete_ = false;
    bool output_uncertain_ = false;
    bool attempt_failed_ = false;
    uint32_t attempt_at_ = 0;
};

bool ValidPosition(const PositionConfig& position_config);
bool ValidOutput(const OutputConfig& pump_config);
bool ConvertPosition(const PositionConfig& position_config, int32_t position_current, int16_t* position_pct, int32_t* position_angle);
bool ConvertPressure(const PressureConfig& pressure_config, int32_t pressure_current, uint32_t* pressure_pa);
bool DriveFresh(const control::Drive& pump_drive, uint32_t now_ms);
bool SameDrive(const control::Drive& pump_first, const control::Drive& pump_second);
bool PlanOutput(const OutputConfig& pump_config, const control::Drive& pump_drive, uint32_t now_ms, OutputPlan* pump_plan);

} // namespace ind::application::io
