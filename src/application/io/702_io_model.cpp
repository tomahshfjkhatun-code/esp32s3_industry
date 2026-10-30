#include "702_io_model.h"

namespace ind::application::io {

namespace {
bool Between(int32_t position_value, int32_t position_first, int32_t position_last) {
    return position_first < position_last ? position_value >= position_first && position_value <= position_last :
                                           position_value <= position_first && position_value >= position_last;
}
int32_t Interpolate(int32_t position_value, int32_t position_first, int32_t position_last, int32_t target_first, int32_t target_last) {
    return static_cast<int32_t>(target_first + static_cast<int64_t>(position_value - position_first) * (target_last - target_first) / (position_last - position_first));
}
}

bool ValidPosition(const PositionConfig& position_config) {
    const int32_t position_low = position_config.position_low;
    const int32_t position_mid = position_config.position_mid;
    const int32_t position_high = position_config.position_high;
    return position_config.position_ready && position_low >= -100000 && position_low <= 100000 &&
        position_mid >= -100000 && position_mid <= 100000 && position_high >= -100000 && position_high <= 100000 &&
        ((position_low < position_mid && position_mid < position_high) || (position_low > position_mid && position_mid > position_high)) &&
        (!position_config.angle_ready || (position_config.angle_low >= -32767 && position_config.angle_high <= 32768 &&
          position_config.angle_low < position_config.angle_mid && position_config.angle_mid < position_config.angle_high));
}

bool ConvertPosition(const PositionConfig& position_config, int32_t position_current, int16_t* position_pct, int32_t* position_angle) {
    if (position_pct == nullptr || position_angle == nullptr || !ValidPosition(position_config) ||
        !Between(position_current, position_config.position_low, position_config.position_high)) return false;
    const bool position_lower = Between(position_current, position_config.position_low, position_config.position_mid);
    const int32_t position_first = position_lower ? position_config.position_low : position_config.position_mid;
    const int32_t position_last = position_lower ? position_config.position_mid : position_config.position_high;
    *position_pct = static_cast<int16_t>(Interpolate(position_current, position_first, position_last, position_lower ? -100 : 0, position_lower ? 0 : 100));
    if (position_config.angle_ready) *position_angle = Interpolate(position_current, position_first, position_last,
        position_lower ? position_config.angle_low : position_config.angle_mid, position_lower ? position_config.angle_mid : position_config.angle_high);
    return true;
}

bool ConvertPressure(const PressureConfig& pressure_config, int32_t pressure_current, uint32_t* pressure_pa) {
    if (pressure_pa == nullptr || !pressure_config.pressure_ready || pressure_config.pressure_low == pressure_config.pressure_high ||
        pressure_config.pressure_low < -100000 || pressure_config.pressure_low > 100000 ||
        pressure_config.pressure_high < -100000 || pressure_config.pressure_high > 100000 ||
        pressure_config.pressure_min >= pressure_config.pressure_max || pressure_config.pressure_max > 65535000U ||
        !Between(pressure_current, pressure_config.pressure_low, pressure_config.pressure_high)) return false;
    *pressure_pa = static_cast<uint32_t>(Interpolate(pressure_current, pressure_config.pressure_low, pressure_config.pressure_high,
        static_cast<int32_t>(pressure_config.pressure_min), static_cast<int32_t>(pressure_config.pressure_max)));
    return true;
}

bool ValidOutput(const OutputConfig& pump_config) {
    return pump_config.output_ready && pump_config.steer_profile_ready &&
        pump_config.steer_frequency > 0 && pump_config.steer_frequency <= 100000 &&
        pump_config.steer_resolution >= 1 && pump_config.steer_resolution <= 14 &&
        pump_config.steer_stop <= 1000 && pump_config.steer_left <= 1000 && pump_config.steer_right <= 1000 &&
        pump_config.steer_stop != pump_config.steer_left && pump_config.steer_stop != pump_config.steer_right &&
        pump_config.steer_left != pump_config.steer_right &&
        (pump_config.steer_pwm_scale == OutputScale::Effort || pump_config.steer_pwm_scale == OutputScale::Fixed);
}

bool DriveFresh(const control::Drive& pump_drive, uint32_t now_ms) {
    return pump_drive.lease_ms > 0 && pump_drive.lease_ms <= 60000 && static_cast<uint32_t>(now_ms - pump_drive.issued_at) <= pump_drive.lease_ms;
}

bool SameDrive(const control::Drive& pump_first, const control::Drive& pump_second) {
    return pump_first.enabled == pump_second.enabled && pump_first.bucket_dir == pump_second.bucket_dir &&
        pump_first.steer_dir == pump_second.steer_dir && pump_first.bucket_effort == pump_second.bucket_effort &&
        pump_first.steer_effort == pump_second.steer_effort && pump_first.bucket_seq == pump_second.bucket_seq &&
        pump_first.steer_seq == pump_second.steer_seq;
}

bool PlanOutput(const OutputConfig& pump_config, const control::Drive& pump_drive, uint32_t now_ms, OutputPlan* pump_plan) {
    if (pump_plan == nullptr || !ValidOutput(pump_config) || pump_drive.bucket_dir < -1 || pump_drive.bucket_dir > 1 ||
        pump_drive.steer_dir < -1 || pump_drive.steer_dir > 1 || pump_drive.bucket_effort > 1000 || pump_drive.steer_effort > 1000 ||
        (pump_drive.bucket_dir != 0 && pump_drive.bucket_effort == 0) || (pump_drive.steer_dir != 0 && pump_drive.steer_effort == 0)) return false;
    OutputPlan pump_next;
    pump_next.bucket_up = !pump_config.bucket_active;
    pump_next.bucket_down = !pump_config.bucket_active;
    pump_next.steer_duty = pump_config.steer_stop;
    pump_next.pump_motion = pump_drive.enabled && DriveFresh(pump_drive, now_ms);
    if (pump_next.pump_motion) {
        const int8_t bucket_direction = pump_config.bucket_reverse ? -pump_drive.bucket_dir : pump_drive.bucket_dir;
        if (bucket_direction > 0) pump_next.bucket_up = pump_config.bucket_active;
        if (bucket_direction < 0) pump_next.bucket_down = pump_config.bucket_active;
        if (pump_drive.steer_dir != 0) {
            const int32_t steer_duty = pump_drive.steer_dir < 0 ? pump_config.steer_left : pump_config.steer_right;
            const uint16_t steer_pwm_effort = pump_config.steer_pwm_scale == OutputScale::Fixed ? 1000 : pump_drive.steer_effort;
            pump_next.steer_duty = static_cast<uint16_t>(pump_config.steer_stop + (steer_duty - pump_config.steer_stop) * steer_pwm_effort / 1000);
        }
    }
    *pump_plan = pump_next;
    return true;
}

bool OutputSequence::Prepare(const OutputConfig& pump_config, const control::Drive& pump_drive, bool output_fault, uint32_t now_ms, uint32_t refresh_ms, uint32_t retry_ms, WriteAction* output_action) {
    if (output_action == nullptr || output_pending_) return false;
    control::Drive output_drive = pump_drive;
    if (output_fault || !output_drive.enabled || !DriveFresh(output_drive, now_ms)) {
        output_drive.enabled = false;
        output_drive.bucket_dir = 0;
        output_drive.steer_dir = 0;
        output_drive.bucket_effort = 0;
        output_drive.steer_effort = 0;
    }
    OutputPlan output_plan;
    if (!PlanOutput(pump_config, output_drive, now_ms, &output_plan)) return false;
    if (!output_seen_ || !SameDrive(output_drive_, output_drive)) {
        output_drive_ = output_drive;
        output_phase_ = 0;
        output_seen_ = true;
        output_failed_ = false;
        output_complete_ = false;
    }
    if (attempt_failed_ && now_ms - attempt_at_ < retry_ms) return false;
    if (output_phase_ >= 4) {
        if (now_ms - attempt_at_ < refresh_ms) return false;
        output_phase_ = output_drive.enabled ? 2 : 0;
        output_failed_ = false;
        output_complete_ = false;
    }
    WriteAction output_next;
    switch (output_phase_) {
    case 0: output_next = {OutputChannel::BucketUp, !pump_config.bucket_active}; break;
    case 1: output_next = {OutputChannel::BucketDown, !pump_config.bucket_active}; break;
    case 2: output_next = {OutputChannel::BucketUp, output_plan.bucket_up}; break;
    case 3: output_next = {OutputChannel::BucketDown, output_plan.bucket_down}; break;
    default: return false;
    }
    *output_action = output_next;
    output_pending_ = true;
    return true;
}

void OutputSequence::Complete(bool output_success, uint32_t now_ms) {
    if (!output_pending_) return;
    output_pending_ = false;
    attempt_at_ = now_ms;
    attempt_failed_ = !output_success;
    output_failed_ = output_failed_ || !output_success;
    output_uncertain_ = output_uncertain_ || !output_success;
    ++output_phase_;
    if (output_phase_ == 4 || (!output_drive_.enabled && output_phase_ == 2)) {
        output_phase_ = 4;
        output_complete_ = !output_failed_;
        if (output_complete_) output_uncertain_ = false;
    }
}

control::Drive OutputSequence::Current() const { return output_drive_; }
bool OutputSequence::Finished() const { return output_complete_; }
bool OutputSequence::Uncertain() const { return output_uncertain_; }

} // namespace ind::application::io
