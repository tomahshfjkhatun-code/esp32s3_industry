// 本文件只用于编译期测试；确认反馈为受控样例，不接入现场设备。
constexpr bool BucketThreePoints() {
    auto bucket_config = ind::application::Make702Config().pump_io.left_pump.bucket_position;
    bucket_config.position_mid = 9000;
    int16_t bucket_pct = 0;
    int32_t bucket_angle = 0;
    if (!pumpio::ConvertPosition(bucket_config, 6500, &bucket_pct, &bucket_angle) || bucket_pct != -50) return false;
    if (!pumpio::ConvertPosition(bucket_config, 14500, &bucket_pct, &bucket_angle) || bucket_pct != 50) return false;
    bucket_config.position_low = 20000;
    bucket_config.position_high = 4000;
    if (!pumpio::ConvertPosition(bucket_config, 14500, &bucket_pct, &bucket_angle) || bucket_pct != -50) return false;
    bucket_config.position_mid = 20000;
    return !pumpio::ConvertPosition(bucket_config, 14500, &bucket_pct, &bucket_angle);
}

constexpr bool SteerIndependentMaps() {
    auto steer_config = OutputConfig();
    auto steer_drive = MotionDrive();
    steer_drive.steer_dir = 1;
    pumpio::OutputPlan steer_plan;
    if (!pumpio::PlanOutput(steer_config, steer_drive, 10, &steer_plan) || steer_plan.steer_duty != 625) return false;
    steer_config.steer_pwm_scale = pumpio::OutputScale::Fixed;
    if (!pumpio::PlanOutput(steer_config, steer_drive, 10, &steer_plan) || steer_plan.steer_duty != 750) return false;
    steer_drive.steer_dir = -1;
    if (!pumpio::PlanOutput(steer_config, steer_drive, 10, &steer_plan) || steer_plan.steer_duty != 250) return false;
    return pumpio::PlanOutput(steer_config, steer_drive, 111, &steer_plan) &&
        steer_plan.steer_duty == 500 && !steer_plan.pump_motion;
}

constexpr bool SteerInvalidMap() {
    auto steer_config = OutputConfig();
    steer_config.steer_pwm_scale = static_cast<pumpio::OutputScale>(99);
    pumpio::OutputPlan steer_plan;
    return !pumpio::PlanOutput(steer_config, MotionDrive(), 10, &steer_plan);
}

constexpr bool PumpCollectionDefault() {
    const auto pump_config = ind::application::Make702Config();
    pumpio::OutputPlan pump_plan;
    AxisControl left_bucket;
    AxisControl left_steer;
    AxisControl right_bucket;
    AxisControl right_steer;
    return left_bucket.Configure(pump_config.left_control.bucket) && left_steer.Configure(pump_config.left_control.steer) &&
        right_bucket.Configure(pump_config.right_control.bucket) && right_steer.Configure(pump_config.right_control.steer) &&
        pump_config.pump_io.left_pump.bucket_position.position_ready && pump_config.pump_io.left_pump.steer_position.position_ready &&
        pump_config.pump_io.right_pump.bucket_position.position_ready && pump_config.pump_io.right_pump.steer_position.position_ready &&
        !pump_config.pump_io.alarm_interlock && !pump_config.pump_io.left_pump.hydraulic_pressure.pressure_ready &&
        !pump_config.pump_io.right_pump.hydraulic_pressure.pressure_ready && !pump_config.pump_io.left_pump.bucket_position.angle_ready &&
        !pump_config.pump_io.left_pump.steer_position.angle_ready && !pump_config.pump_io.right_pump.bucket_position.angle_ready &&
        !pump_config.pump_io.right_pump.steer_position.angle_ready &&
        !pump_config.pump_io.left_pump.valve_position.position_ready &&
        !pump_config.pump_io.right_pump.valve_position.position_ready && !pump_config.alarm_routes.enabled &&
        !pumpio::PlanOutput(pump_config.pump_io.left_pump.pump_output, MotionDrive(), 10, &pump_plan) &&
        !pumpio::PlanOutput(pump_config.pump_io.right_pump.pump_output, MotionDrive(), 10, &pump_plan);
}

// 这些对象仅作为 Runtime 的引用载体；本测试不调用虚拟 IO 或 CAN 方法。
class PumpIoFixture final : public IoPort {
public:
    ReadResult Read(Side, IoSample*, uint32_t*) override { return ReadResult::Empty; }
    bool Apply(Side, const Drive&) override { return false; }
};
class PumpCanFixture final : public CanPort {
public:
    bool Publish(const Snapshot&) override { return false; }
};
extern PumpIoFixture pump_io_fixture;
extern PumpCanFixture pump_can_fixture;

constexpr bool PumpSeparateSettings() {
    auto pump_config = ind::application::Make702Config();
    pump_config.left_control.bucket.effort = 111;
    pump_config.left_control.steer.effort = 222;
    pump_config.right_control.bucket.effort = 333;
    pump_config.right_control.steer.effort = 444;
    PumpRuntime pump_runtime(pump_io_fixture, pump_can_fixture);
    if (!pump_runtime.Configure(pump_config.left_control, pump_config.right_control)) return false;
    if (pump_runtime.Configure(pump_config.left_control)) return false;
    return pump_runtime.Configure(pump_config.left_control, pump_config.right_control);
}

constexpr bool PumpAtomicSettings() {
    auto pump_config = ind::application::Make702Config();
    PumpRuntime pump_runtime(pump_io_fixture, pump_can_fixture);
    pump_config.right_control.steer.effort = 1001;
    if (pump_runtime.Configure(pump_config.left_control, pump_config.right_control)) return false;
    pump_config.right_control.steer.effort = 444;
    pump_config.left_control.steer.effort = 111;
    if (!pump_runtime.Configure(pump_config.left_control, pump_config.right_control)) return false;
    pump_config.right_control.stop_policy = StopPolicy::Independent;
    return !pump_runtime.Configure(pump_config.left_control, pump_config.right_control);
}

constexpr pumpio::Io702 PumpInputFixture() {
    pumpio::Io702 pump_io;
    pump_io.pump_config_ = ind::application::Make702Config().pump_io;
    pump_io.io_status_.network_ready = true;
    auto& left_pump = pump_io.io_status_.left_pump;
    left_pump.bucket_current.attempt_at = 90;
    left_pump.bucket_current.at_ms = 90;
    left_pump.steer_current.attempt_at = 95;
    left_pump.steer_current.at_ms = 95;
    left_pump.bucket_position = {20, 90, true};
    left_pump.steer_position = {-30, 95, true};
    return pump_io;
}

constexpr bool ValveRequired() {
    auto pump_io = PumpInputFixture();
    auto& status = pump_io.io_status_.left_pump;
    auto& config = pump_io.pump_config_.left_pump;
    config.pump_output.output_ready = true;
    IoSample sample;
    uint32_t stamp = 0;
    if (pump_io.Read(Side::Left, &sample, &stamp) != ReadResult::Empty) return false;
    config.valve_position.position_ready = true;
    status.valve_current.attempt_at = 90;
    if (pump_io.Read(Side::Left, &sample, &stamp) != ReadResult::Sample || !sample.input_fault) return false;
    status.valve_current.at_ms = 90;
    status.valve_position = {20, 90, true};
    return pump_io.Read(Side::Left, &sample, &stamp) == ReadResult::Sample &&
        sample.valid && sample.valve_valid && sample.valve_pct == 20 && stamp == 90;
}

constexpr bool ValveAiConversion() {
    auto pump_io = PumpInputFixture();
    auto& status = pump_io.io_status_.left_pump;
    auto& config = pump_io.pump_config_.left_pump;
    config.valve_position.position_ready = true;
    status.valve_current.value = 12000;
    status.valve_current.at_ms = 90;
    status.valve_current.valid = true;
    pump_io.Convert(status, config, 100);
    if (!status.valve_position.valid || status.valve_position.value != 0) return false;
    pump_io.Convert(status, config, 1100);
    return !status.valve_position.valid;
}

constexpr bool PumpAlarmCollection() {
    auto pump_io = PumpInputFixture();
    IoSample pump_sample;
    uint32_t sample_at = 0;
    // 报警尚未读到也不阻塞位置；软硬限位未提供时继续保持无效。
    if (pump_io.Read(Side::Left, &pump_sample, &sample_at) != ReadResult::Sample ||
        !pump_sample.valid || pump_sample.mechanism_fault || sample_at != 90 || pump_sample.bucket_limits.valid) return false;
    pump_io.io_status_.main_oil_low.value = true;
    pump_io.io_status_.main_oil_low.valid = true;
    pump_io.io_status_.main_oil_low.at_ms = 1;
    return pump_io.Read(Side::Left, &pump_sample, &sample_at) == ReadResult::Sample &&
        !pump_sample.mechanism_fault && sample_at == 90 && pump_sample.bucket_pct == 20 && pump_sample.steer_pct == -30;
}

constexpr bool PumpAlarmInterlock() {
    auto pump_io = PumpInputFixture();
    pump_io.pump_config_.alarm_interlock = true;
    IoSample pump_sample;
    uint32_t sample_at = 0;
    if (pump_io.Read(Side::Left, &pump_sample, &sample_at) != ReadResult::Empty) return false;
    pump_io.pump_config_.alarms_ready = true;
    if (pump_io.Read(Side::Left, &pump_sample, &sample_at) != ReadResult::Empty) return false;
    auto& pump_status = pump_io.io_status_;
    pump_status.main_oil_low.attempt_at = pump_status.main_oil_hot.attempt_at = pump_status.left_pump.filter_alarm.attempt_at = 80;
    if (pump_io.Read(Side::Left, &pump_sample, &sample_at) != ReadResult::Sample || !pump_sample.input_fault) return false;
    pump_status.main_oil_low.valid = pump_status.main_oil_hot.valid = pump_status.left_pump.filter_alarm.valid = true;
    pump_status.main_oil_low.at_ms = pump_status.main_oil_hot.at_ms = pump_status.left_pump.filter_alarm.at_ms = 80;
    pump_status.main_oil_low.value = true;
    return pump_io.Read(Side::Left, &pump_sample, &sample_at) == ReadResult::Sample &&
        pump_sample.valid && pump_sample.mechanism_fault && sample_at == 80;
}

constexpr bool PumpPositionRequired() {
    auto pump_io = PumpInputFixture();
    pump_io.io_status_.left_pump.steer_position.valid = false;
    IoSample pump_sample;
    uint32_t sample_at = 0;
    return pump_io.Read(Side::Left, &pump_sample, &sample_at) == ReadResult::Sample &&
        pump_sample.input_fault && !pump_sample.valid;
}

static_assert(BucketThreePoints(), "asymmetric and descending calibration; reject duplicate points");
static_assert(SteerIndependentMaps(), "PWM direction endpoints and expiry stop");
static_assert(SteerInvalidMap(), "unknown output mapping rejected");
static_assert(PumpCollectionDefault(), "four provisional profiles valid; hardware and unknown physical data disabled");
static_assert(PumpSeparateSettings(), "runtime retains distinct parameters for four axes");
static_assert(PumpAtomicSettings(), "bad right profile must not partially lock left profile");
static_assert(PumpAlarmCollection(), "monitor-only alarms do not affect position or its timestamp");
static_assert(ValveAiConversion(), "valve AI creates fresh position and expires when stale");
static_assert(ValveRequired(), "enabled output requires valid valve AI feedback");
static_assert(ind::application::io::map::ai::left_valve_pos == 0 && ind::application::io::map::ai::left_bucket_pos == 1 &&
    ind::application::io::map::ai::left_steer_pos == 2 && ind::application::io::map::ai::left_pressure == 3 &&
    ind::application::io::map::ai::right_valve_pos == 4 && ind::application::io::map::ai::right_bucket_pos == 5 &&
    ind::application::io::map::ai::right_steer_pos == 6 && ind::application::io::map::ai::right_pressure == 7, "eight AI channels use the agreed provisional order");
static_assert(PumpAlarmInterlock(), "opt-in alarm interlock still requires configuration and valid samples");
static_assert(PumpPositionRequired(), "monitor-only alarms do not bypass required position feedback");
