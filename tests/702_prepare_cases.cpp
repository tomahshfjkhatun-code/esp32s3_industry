using namespace ind::application::control;
namespace protocol = ind::application::can;
namespace pumpio = ind::application::io;

constexpr AxisSettings AxisConfig() {
    AxisSettings axis_config;
    axis_config.min_pct = -100;
    axis_config.max_pct = 100;
    axis_config.start_tol = 3;
    axis_config.stop_tol = 1;
    axis_config.min_progress = 1;
    axis_config.wrong_tol = 3;
    axis_config.effort = 500;
    axis_config.ack_ms = 15;
    axis_config.travel_ms = 100;
    axis_config.stall_ms = 20;
    axis_config.grace_ms = 5;
    axis_config.reverse_ms = 10;
    return axis_config;
}

constexpr bool CommandFields() {
    uint8_t pump_data[8] = {27, 227, 100, 5, 0x99, 0, 0, 0};
    protocol::Command pump_command;
    return protocol::DecodeCommand(0x04069082, 8, pump_data, &pump_command) &&
        pump_command.right_pump && pump_command.bucket_target == -100 && pump_command.steer_target == 100 &&
        pump_command.throttle_target == 100 && pump_command.clutch_request && pump_command.backflush_request &&
        pump_command.engine_action == protocol::EngineAction::Start && pump_command.engine_estop == protocol::SwitchAction::Off &&
        pump_command.engine_warm == protocol::SwitchAction::On && pump_command.engine_override == protocol::SwitchAction::Off;
}
constexpr bool CommandBounds() {
    uint8_t pump_data[8] = {127, 127, 0, 0, 0, 0, 0, 0};
    protocol::Command pump_command;
    if (protocol::DecodeCommand(0x04069002, 7, pump_data, &pump_command) || protocol::DecodeCommand(4, 8, pump_data, &pump_command)) return false;
    pump_data[0] = 26;
    if (protocol::DecodeCommand(0x04069002, 8, pump_data, &pump_command)) return false;
    pump_data[0] = 127;
    pump_data[1] = 228;
    if (protocol::DecodeCommand(0x04069002, 8, pump_data, &pump_command)) return false;
    pump_data[1] = 127;
    pump_data[2] = 101;
    if (protocol::DecodeCommand(0x04069002, 8, pump_data, &pump_command)) return false;
    pump_data[2] = 100;
    pump_data[4] = 3;
    return !protocol::DecodeCommand(0x04069002, 8, pump_data, &pump_command);
}
constexpr bool AuthorityBounds() {
    uint8_t pump_data[8] = {1, 255, 255, 255, 255, 255, 255, 255};
    protocol::Command pump_command;
    if (!protocol::DecodeCommand(0x04069001, 8, pump_data, &pump_command)) return false;
    pump_data[0] = 3;
    if (protocol::DecodeCommand(0x04069001, 8, pump_data, &pump_command)) return false;
    pump_data[0] = 2;
    pump_data[7] = 0;
    return !protocol::DecodeCommand(0x04069001, 8, pump_data, &pump_command);
}
constexpr bool CommandRetention() {
    PumpControl left_pump(Side::Left);
    Settings pump_config;
    pump_config.auth_ms = 600;
    pump_config.motion_ms = 150;
    pump_config.io_ms = 1000;
    pump_config.stop_policy = StopPolicy::Together;
    if (!left_pump.Configure(pump_config)) return false;
    uint8_t auth_data[8] = {1, 255, 255, 255, 255, 255, 255, 255};
    uint8_t motion_data[8] = {127, 180, 53, 5, 1, 0, 0, 0};
    if (left_pump.ReceiveCan(0x04069002, 8, motion_data, 1)) return false;
    if (!left_pump.ReceiveCan(0x04069001, 8, auth_data, 2) || !left_pump.ReceiveCan(0x04069002, 8, motion_data, 3)) return false;
    left_pump.ReceiveCan(0x04069001, 8, auth_data, 4);
    const auto pump_status = left_pump.Status();
    if (pump_status.motion_command.pump_command.throttle_target != 53 || !pump_status.motion_command.command_accepted) return false;
    if (left_pump.ReceiveCan(0x04069082, 8, motion_data, 5)) return false;
    auth_data[0] = 2;
    left_pump.ReceiveCan(0x04069001, 8, auth_data, 6);
    return !left_pump.ReceiveCan(0x04069002, 8, motion_data, 7) && !left_pump.Step(7).enabled;
}
constexpr bool AuthorityExpiry() {
    PumpControl left_pump(Side::Left);
    Settings pump_config;
    pump_config.auth_ms = 10;
    pump_config.motion_ms = 5;
    pump_config.io_ms = 10;
    pump_config.stop_policy = StopPolicy::Independent;
    left_pump.Configure(pump_config);
    uint8_t auth_data[8] = {1, 255, 255, 255, 255, 255, 255, 255};
    uint8_t motion_data[8] = {127, 127, 0, 0, 0, 0, 0, 0};
    left_pump.ReceiveCan(0x04069001, 8, auth_data, 1);
    if (left_pump.ReceiveCan(0x04069002, 8, motion_data, 12)) return false;
    left_pump.ReceiveCan(0x04069001, 8, auth_data, 13);
    left_pump.Step(13);
    return left_pump.Status().motion_age == UINT32_MAX;
}
constexpr bool FaultRevokes() {
    PumpControl left_pump(Side::Left);
    Settings pump_config;
    pump_config.auth_ms = 100;
    pump_config.motion_ms = 50;
    pump_config.io_ms = 100;
    pump_config.stop_policy = StopPolicy::Independent;
    left_pump.Configure(pump_config);
    uint8_t auth_data[8] = {1, 255, 255, 255, 255, 255, 255, 255};
    left_pump.ReceiveCan(0x04069001, 8, auth_data, 1);
    left_pump.ReportFault(FaultCode::CanBus, 2);
    return !left_pump.ReceiveCan(0x04069001, 8, auth_data, 3) && !left_pump.Status().unmanned && !left_pump.Step(4).enabled;
}
constexpr protocol::ProtocolSettings ProtocolConfig() {
    protocol::ProtocolSettings pump_config;
    pump_config.status_ms = 100;
    pump_config.speed_layout = true;
    pump_config.reserved_layout = true;
    return pump_config;
}
constexpr bool PressureBytes() {
    protocol::PumpStatus pump_status;
    pump_status.gearbox_pressure = {466000, 10, true};
    pump_status.gearbox_temperature = {-10, 10, true};
    pump_status.hydraulic_pressure = {258000, 10, true};
    pump_status.gearbox_lubrication = {6553500, 10, true};
    protocol::StatusFrame pump_frame;
    if (protocol::EncodeStatus(true, 1, pump_status, ProtocolConfig(), 10, &pump_frame) != protocol::EncodeResult::Ready) return false;
    return pump_frame.id == 0x06069082 && pump_frame.data[0] == 0x12 && pump_frame.data[1] == 0x34 &&
        pump_frame.data[2] == 0x7F && pump_frame.data[3] == 0xF5 && pump_frame.data[4] == 1 &&
        pump_frame.data[5] == 2 && pump_frame.data[6] == 255 && pump_frame.data[7] == 255;
}
constexpr bool AlarmRouteMap() {
    protocol::AlarmRoute route;
    protocol::Signal<bool> input{false, 10, true};
    protocol::Signal<protocol::Alarm> output;
    if (protocol::MapAlarm(route, input, &output) || output.valid) return false;
    route.ready = true;
    route.device_type = 2;
    route.spn = 0x12345;
    route.fmi = 4;
    route.active_level = false;
    if (!protocol::MapAlarm(route, input, &output) || !output.valid || !output.value.alarm_active ||
        output.at_ms != 10 || output.value.alarm_spn != 0x12345) return false;
    input.value = true;
    if (!protocol::MapAlarm(route, input, &output) || output.value.alarm_active) return false;
    input.valid = false;
    return !protocol::MapAlarm(route, input, &output) && !output.valid;
}

constexpr bool AlarmPairs() {
    protocol::AlarmRoute low{true, 2, 100, 4, true};
    protocol::AlarmRoute hot{true, 2, 101, 5, false};
    protocol::AlarmRoute filter{true, 2, 102, 6, true};
    protocol::Signal<bool> input{true, 10, true};
    protocol::PumpStatus status;
    protocol::StatusFrame first;
    protocol::StatusFrame second;
    if (!protocol::MapAlarm(low, input, &status.pump_alarms[0])) return false;
    input.value = false;
    if (!protocol::MapAlarm(hot, input, &status.pump_alarms[1])) return false;
    if (protocol::EncodeStatus(false, 3, status, ProtocolConfig(), 10, &first) != protocol::EncodeResult::Ready) return false;
    input.value = true;
    if (!protocol::MapAlarm(filter, input, &status.pump_alarms[0])) return false;
    if (!protocol::MapAlarm(filter, input, &status.pump_alarms[1])) return false;
    if (protocol::EncodeStatus(false, 3, status, ProtocolConfig(), 10, &second) != protocol::EncodeResult::Ready) return false;
    return first.id == second.id && first.id == 0x0A069001U &&
        first.data[1] == 100 && first.data[5] == 101 && second.data[1] == 102 && second.data[5] == 102 &&
        (first.data[0] & 4U) != 0 && (first.data[4] & 4U) != 0;
}

constexpr bool AlarmBytes() {
    protocol::PumpStatus pump_status;
    pump_status.pump_alarms[0] = {{2, true, 0x51234, 7}, 10, true};
    pump_status.pump_alarms[1] = {{3, false, 0x123, 31}, 10, true};
    protocol::StatusFrame pump_frame;
    if (protocol::EncodeStatus(false, 3, pump_status, ProtocolConfig(), 10, &pump_frame) != protocol::EncodeResult::Ready) return false;
    return pump_frame.id == 0x0A069001 && pump_frame.data[0] == 0xF6 && pump_frame.data[1] == 0x34 &&
        pump_frame.data[2] == 0x12 && pump_frame.data[3] == 0xA7 && pump_frame.data[4] == 0xF3 &&
        pump_frame.data[5] == 0x23 && pump_frame.data[6] == 1 && pump_frame.data[7] == 31;
}
constexpr bool PositionBytes() {
    protocol::PumpStatus pump_status;
    pump_status.bucket_angle = {-100, 10, true};
    pump_status.steer_angle = {100, 10, true};
    pump_status.clutch_engaged = {true, 10, true};
    pump_status.backflush_active = {false, 10, true};
    protocol::StatusFrame pump_frame;
    if (protocol::EncodeStatus(false, 0, pump_status, ProtocolConfig(), 10, &pump_frame) != protocol::EncodeResult::Ready) return false;
    return pump_frame.data[0] == 0x7F && pump_frame.data[1] == 0x9B && pump_frame.data[2] == 0x80 &&
        pump_frame.data[3] == 0x63 && pump_frame.data[4] == 0xF1 && pump_frame.data[5] == 0 &&
        pump_frame.data[6] == 255 && pump_frame.data[7] == 255;
}
constexpr bool MissingStatus() {
    protocol::PumpStatus pump_status;
    protocol::StatusFrame pump_frame;
    pump_frame.id = 123;
    if (protocol::EncodeStatus(false, 1, pump_status, ProtocolConfig(), 10, &pump_frame) != protocol::EncodeResult::NotReady || pump_frame.id != 123) return false;
    pump_status.valve_position = {0, 10, true};
    pump_status.hydraulic_running = {false, 10, true};
    if (protocol::EncodeStatus(false, 2, pump_status, ProtocolConfig(), 111, &pump_frame) != protocol::EncodeResult::NotReady) return false;
    if (protocol::EncodeStatus(false, 2, pump_status, protocol::ProtocolSettings{}, 10, &pump_frame) != protocol::EncodeResult::NotReady) return false;
    pump_status.valve_position.value = 101;
    return protocol::EncodeStatus(false, 2, pump_status, ProtocolConfig(), 10, &pump_frame) == protocol::EncodeResult::Invalid;
}
constexpr bool PositionConversion() {
    pumpio::PositionConfig bucket_config;
    int16_t bucket_pct = 9;
    int32_t bucket_angle = 9;
    if (pumpio::ConvertPosition(bucket_config, 12000, &bucket_pct, &bucket_angle)) return false;
    bucket_config.position_ready = true;
    bucket_config.angle_ready = true;
    bucket_config.angle_low = -3000;
    bucket_config.angle_mid = 0;
    bucket_config.angle_high = 4500;
    if (!pumpio::ConvertPosition(bucket_config, 16000, &bucket_pct, &bucket_angle) || bucket_pct != 50 || bucket_angle != 2250) return false;
    if (pumpio::ConvertPosition(bucket_config, 3999, &bucket_pct, &bucket_angle)) return false;
    bucket_config.position_low = 20000;
    bucket_config.position_high = 4000;
    return pumpio::ConvertPosition(bucket_config, 16000, &bucket_pct, &bucket_angle) && bucket_pct == -50 && bucket_angle == -1500;
}
constexpr bool PressureConversion() {
    pumpio::PressureConfig pressure_config;
    pressure_config.pressure_ready = true;
    pressure_config.pressure_max = 2000000;
    uint32_t pressure_value = 0;
    if (!pumpio::ConvertPressure(pressure_config, 12000, &pressure_value) || pressure_value != 1000000) return false;
    return !pumpio::ConvertPressure(pressure_config, 20001, &pressure_value);
}
constexpr pumpio::OutputConfig OutputConfig() {
    pumpio::OutputConfig pump_config;
    pump_config.output_ready = true;
    pump_config.steer_profile_ready = true;
    pump_config.steer_frequency = 1000;
    pump_config.steer_stop = 500;
    pump_config.steer_left = 250;
    pump_config.steer_right = 750;
    return pump_config;
}
constexpr Drive MotionDrive() {
    Drive pump_drive;
    pump_drive.enabled = true;
    pump_drive.bucket_dir = 1;
    pump_drive.steer_dir = -1;
    pump_drive.bucket_effort = 500;
    pump_drive.steer_effort = 500;
    pump_drive.bucket_seq = 1;
    pump_drive.steer_seq = 2;
    pump_drive.issued_at = 10;
    pump_drive.lease_ms = 100;
    return pump_drive;
}
constexpr bool OutputMapping() {
    auto pump_config = OutputConfig();
    const auto pump_drive = MotionDrive();
    pumpio::OutputPlan pump_plan;
    if (!pumpio::PlanOutput(pump_config, pump_drive, 10, &pump_plan)) return false;
    if (!pump_plan.bucket_up || pump_plan.bucket_down || pump_plan.steer_duty != 375) return false;
    pump_config.bucket_reverse = true;
    pump_config.bucket_active = false;
    if (!pumpio::PlanOutput(pump_config, pump_drive, 10, &pump_plan) || !pump_plan.bucket_up || pump_plan.bucket_down) return false;
    return pumpio::PlanOutput(pump_config, pump_drive, 111, &pump_plan) && pump_plan.bucket_up && pump_plan.bucket_down &&
        pump_plan.steer_duty == 500;
}
constexpr bool LeaseWrap() {
    auto pump_drive = MotionDrive();
    pump_drive.issued_at = UINT32_MAX - 5U;
    pump_drive.lease_ms = 10;
    return pumpio::DriveFresh(pump_drive, 4) && !pumpio::DriveFresh(pump_drive, 5);
}
constexpr bool SafeSequence() {
    pumpio::OutputSequence pump_sequence;
    pumpio::WriteAction pump_action;
    const auto pump_drive = MotionDrive();
    const pumpio::OutputChannel channels[4] = {pumpio::OutputChannel::BucketUp, pumpio::OutputChannel::BucketDown, pumpio::OutputChannel::BucketUp, pumpio::OutputChannel::BucketDown};
    const int32_t values[4] = {0, 0, 1, 0};
    for (uint8_t pump_index = 0; pump_index < 4; ++pump_index) {
        if (!pump_sequence.Prepare(OutputConfig(), pump_drive, false, 10 + pump_index, 50, 5, &pump_action)) return false;
        if (pump_action.output_channel != channels[pump_index] || pump_action.output_value != values[pump_index]) return false;
        pump_sequence.Complete(true, 10 + pump_index);
    }
    if (!pump_sequence.Finished()) return false;
    if (pump_sequence.Prepare(OutputConfig(), pump_drive, false, 14, 50, 5, &pump_action)) return false;
    return pump_sequence.Prepare(OutputConfig(), pump_drive, false, 63, 50, 5, &pump_action) &&
        pump_action.output_channel == pumpio::OutputChannel::BucketUp && pump_action.output_value == 1;
}
constexpr bool StopAfterFailure() {
    pumpio::OutputSequence pump_sequence;
    pumpio::WriteAction pump_action;
    auto pump_drive = MotionDrive();
    pump_drive.enabled = false;
    if (!pump_sequence.Prepare(OutputConfig(), pump_drive, false, 10, 50, 5, &pump_action)) return false;
    pump_sequence.Complete(false, 10);
    if (pump_sequence.Prepare(OutputConfig(), pump_drive, true, 11, 50, 5, &pump_action)) return false;
    if (!pump_sequence.Prepare(OutputConfig(), pump_drive, true, 15, 50, 5, &pump_action) ||
        pump_action.output_channel != pumpio::OutputChannel::BucketDown || pump_action.output_value != 0) return false;
    pump_sequence.Complete(true, 15);
    return !pump_sequence.Finished() && pump_sequence.Uncertain();
}
constexpr bool SupersededOutput() {
    pumpio::OutputSequence pump_sequence;
    pumpio::WriteAction pump_action;
    auto pump_drive = MotionDrive();
    for (uint8_t pump_index = 0; pump_index < 4; ++pump_index) {
        pump_sequence.Prepare(OutputConfig(), pump_drive, false, 10 + pump_index, 50, 5, &pump_action);
        pump_sequence.Complete(true, 10 + pump_index);
    }
    pump_drive.bucket_dir = -1;
    pump_drive.bucket_seq = 2;
    if (!pump_sequence.Prepare(OutputConfig(), pump_drive, false, 14, 50, 5, &pump_action) ||
        pump_action.output_channel != pumpio::OutputChannel::BucketUp || pump_action.output_value != 0) return false;
    pump_sequence.Complete(true, 14);
    return pump_sequence.Prepare(OutputConfig(), pump_drive, false, 111, 50, 5, &pump_action) &&
        !pump_sequence.Current().enabled && pump_action.output_channel == pumpio::OutputChannel::BucketUp && pump_action.output_value == 0;
}

static_assert(CommandFields(), "all motion fields and side");
static_assert(CommandBounds(), "invalid frames cannot update commands");
static_assert(AuthorityBounds(), "authority validation");
static_assert(CommandRetention(), "authority and motion histories stay separate");
static_assert(AuthorityExpiry(), "expired authority cannot retain motion");
static_assert(FaultRevokes(), "bus fault revokes authorization");
static_assert(PressureBytes(), "pressure golden frame");
static_assert(AlarmBytes(), "SPN uses explicit table bit order");
static_assert(AlarmRouteMap(), "DI level maps to configured alarm and stale input stays invalid");
static_assert(AlarmPairs(), "multiple alarm frames retain the same CAN ID and distinct SPN values");
static_assert(PositionBytes(), "angle offset and missing RPM sentinel");
static_assert(MissingStatus(), "missing/stale/unconfirmed status not sent");
static_assert(PositionConversion(), "calibration direction and out of range");
static_assert(PressureConversion(), "pressure units and range");
static_assert(OutputMapping(), "polarity/direction/effort and stale stop");
static_assert(LeaseWrap(), "millisecond wrap");
static_assert(SafeSequence(), "neutral before enabling and keepalive");
static_assert(StopAfterFailure(), "failure must not starve remaining safe writes");
static_assert(SupersededOutput(), "old sequence cannot continue after reversal or expiry");

constexpr bool DefaultDisabled() {
    pumpio::OutputPlan pump_plan;
    PumpControl left_pump(Side::Left);
    return !pumpio::PlanOutput(pumpio::OutputConfig{}, MotionDrive(), 10, &pump_plan) &&
        !left_pump.Step(10).enabled && left_pump.Status().wait == WaitReason::Configuration;
}
constexpr bool FailedAction() {
    pumpio::OutputSequence pump_sequence;
    pumpio::WriteAction pump_action;
    const auto pump_drive = MotionDrive();
    for (uint8_t pump_index = 0; pump_index < 3; ++pump_index) {
        pump_sequence.Prepare(OutputConfig(), pump_drive, false, 10 + pump_index, 50, 5, &pump_action);
        pump_sequence.Complete(true, 10 + pump_index);
    }
    pump_sequence.Prepare(OutputConfig(), pump_drive, false, 13, 50, 5, &pump_action);
    pump_sequence.Complete(false, 13);
    if (pump_sequence.Prepare(OutputConfig(), pump_drive, true, 14, 50, 5, &pump_action)) return false;
    return pump_sequence.Prepare(OutputConfig(), pump_drive, true, 18, 50, 5, &pump_action) &&
        !pump_sequence.Current().enabled && pump_action.output_channel == pumpio::OutputChannel::BucketUp &&
        pump_action.output_value == 0 && pump_sequence.Uncertain();
}
static_assert(DefaultDisabled(), "default configuration cannot energize outputs");
static_assert(FailedAction(), "failed action cannot be retried as movement");
