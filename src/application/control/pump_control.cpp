#include "pump_control.h"

namespace ind::application::control {

namespace {

constexpr uint32_t left_auth = 0x04069001U;
constexpr uint32_t right_auth = 0x04069081U;
constexpr uint32_t left_motion = 0x04069002U;
constexpr uint32_t right_motion = 0x04069082U;

} // namespace

PumpControl::PumpControl(Side side) : side_(side) {}

bool PumpControl::Configure(const Settings& cfg) {
    if (cfg.auth_ms == 0 || cfg.motion_ms == 0 || cfg.io_ms == 0 ||
        cfg.auth_ms > 60000 || cfg.motion_ms > 60000 || cfg.io_ms > 60000 ||
        (cfg.stop_policy != StopPolicy::Independent && cfg.stop_policy != StopPolicy::Together)) {
        return false;
    }
    if (configured_) {
        if (cfg.auth_ms != cfg_.auth_ms || cfg.motion_ms != cfg_.motion_ms ||
            cfg.io_ms != cfg_.io_ms || cfg.stop_policy != cfg_.stop_policy) return false;
    }
    // 参数全零允许仅运行 CAN；轴未就绪时保持 Configuration 等待。部分填写但无效则拒绝。
    AxisControl bucket = bucket_;
    AxisControl steer = steer_;
    if (!bucket.Configure(cfg.bucket) || !steer.Configure(cfg.steer)) return false;
    bucket_ = bucket;
    steer_ = steer;
    cfg_ = cfg;
    configured_ = true;
    return true;
}

bool PumpControl::ValidCan(uint32_t id, uint8_t dlc, const uint8_t data[8]) {
    can::Command pump_command;
    return can::DecodeCommand(id, dlc, data, &pump_command);
}

void PumpControl::Expire(uint32_t now_ms) {
    if (!Fresh(auth_seen_, auth_at_, cfg_.auth_ms, now_ms)) {
        auth_seen_ = false;
        unmanned_ = false;
        motion_seen_ = false;
    }
}

bool PumpControl::ReceiveCan(uint32_t id, uint8_t dlc, const uint8_t data[8], uint32_t now_ms) {
    can::Command pump_command;
    if (!can::DecodeCommand(id, dlc, data, &pump_command)) return false;
    const uint32_t auth_id = side_ == Side::Left ? left_auth : right_auth;
    const uint32_t motion_id = side_ == Side::Left ? left_motion : right_motion;
    if (id != auth_id && id != motion_id) return false;
    auto& command_status = id == auth_id ? authority_command_ : motion_command_;
    command_status = {pump_command, now_ms, true, false};
    // 先检查旧授权，防止同一轮出队中的续授权报文覆盖已超时的授权时间。
    Expire(now_ms);
    if (faults_ != 0) return false;
    if (id == auth_id) {
        const uint8_t priority = data[0] & 0x03U;
        const bool next = priority == 1;
        if (next != unmanned_) {
            motion_seen_ = false;
        }
        unmanned_ = next;
        auth_at_ = now_ms;
        auth_seen_ = true;
        command_status.command_accepted = true;
        return true;
    }

    if (id == motion_id) {
        // 未授权、有人优先以及故障期间的运动帧都不能留作将来的动作目标。
        if (!unmanned_ || !auth_seen_) return false;
        const int16_t bucket = static_cast<int16_t>(data[0]) - 127;
        const int16_t steer = static_cast<int16_t>(data[1]) - 127;
        if (bucket < -100 || bucket > 100 || steer < -100 || steer > 100 ||
            data[2] > 100 || (data[4] & 0x03U) == 0x03U) {
            return false;
        }
        // 全部指令字段保存在 motion_command_；附属设备没有通道映射时不产生输出。
        command_status.command_accepted = true;
        bucket_target_ = bucket;
        steer_target_ = steer;
        motion_at_ = now_ms;
        motion_seen_ = true;
        return true;
    }

    return false;
}

void PumpControl::ReceiveIo(const IoSample& sample, uint32_t now_ms) {
    io_ = sample;
    io_at_ = now_ms;
    io_seen_ = true;
    if (sample.mechanism_fault) ReportFault(FaultCode::Mechanism, now_ms);
    if (sample.input_fault || !sample.valid || sample.bucket_pct < -100 ||
        sample.bucket_pct > 100 || sample.steer_pct < -100 || sample.steer_pct > 100) {
        io_.valid = false;
        ReportFault(FaultCode::Input, now_ms);
    }
}

void PumpControl::ReceiveOutput(const OutputSample& sample) {
    output_ = sample;
}

void PumpControl::ReportFault(FaultCode fault, uint32_t now_ms) {
    if (faults_ == 0) fault_at_ = now_ms;
    faults_ |= static_cast<uint32_t>(fault);
    auth_seen_ = false;
    motion_seen_ = false;
    unmanned_ = false;
    bucket_.Stop(now_ms);
    steer_.Stop(now_ms);
    UpdateDrive(now_ms);
    mode_ = Mode::Fault;
    wait_ = WaitReason::None;
}

void PumpControl::Inhibit(WaitReason reason) {
    bucket_.Stop(now_ms_);
    steer_.Stop(now_ms_);
    UpdateDrive(now_ms_);
    motion_seen_ = false;
    if (faults_ == 0) {
        mode_ = Mode::Waiting;
        wait_ = reason;
    }
}

bool PumpControl::Fresh(bool seen, uint32_t stamp, uint32_t limit, uint32_t now_ms) const {
    return seen && static_cast<uint32_t>(now_ms - stamp) <= limit;
}

void PumpControl::UpdateDrive(uint32_t now_ms) {
    const AxisDemand bucket = bucket_.Status().demand;
    const AxisDemand steer = steer_.Status().demand;
    drive_.bucket_dir = bucket.direction;
    drive_.steer_dir = steer.direction;
    drive_.bucket_effort = bucket.effort;
    drive_.steer_effort = steer.effort;
    drive_.bucket_seq = bucket.sequence;
    drive_.steer_seq = steer.sequence;
    drive_.enabled = bucket.direction != 0 || steer.direction != 0;
    drive_.issued_at = now_ms;
    drive_.lease_ms = cfg_.io_ms;
}

void PumpControl::CheckAxes(uint32_t now_ms) {
    const uint32_t faults = bucket_.Status().faults | steer_.Status().faults;
    const uint32_t output = static_cast<uint32_t>(AxisFault::StartTimeout) | static_cast<uint32_t>(AxisFault::StopTimeout) | static_cast<uint32_t>(AxisFault::FeedbackLost);
    const uint32_t input = static_cast<uint32_t>(AxisFault::Input);
    if ((faults & output) != 0) ReportFault(FaultCode::Output, now_ms);
    if ((faults & input) != 0) ReportFault(FaultCode::Input, now_ms);
    if ((faults & ~(output | input)) != 0) ReportFault(FaultCode::Motion, now_ms);
}

Drive PumpControl::Step(uint32_t now_ms) {
    now_ms_ = now_ms;
    Expire(now_ms);
    wait_ = WaitReason::None;
    if (!configured_ || !bucket_.Ready() || !steer_.Ready()) wait_ = WaitReason::Configuration;
    else if (!unmanned_ || !auth_seen_) wait_ = WaitReason::Authority;
    else if (!Fresh(io_seen_, io_at_, cfg_.io_ms, now_ms) || !io_.valid || !io_.bucket_limits.valid || !io_.steer_limits.valid) {
        wait_ = WaitReason::Input;
        motion_seen_ = false;
    }
    else if (!Fresh(motion_seen_, motion_at_, cfg_.motion_ms, now_ms)) wait_ = WaitReason::Motion;
    const bool allow = faults_ == 0 && wait_ == WaitReason::None;
    const bool fresh = Fresh(io_seen_, io_at_, cfg_.io_ms, now_ms) && io_.valid;
    bucket_.Step(allow, bucket_target_, AxisInput{io_.bucket_pct, io_at_, fresh, io_.bucket_limits}, output_.bucket, now_ms);
    steer_.Step(allow, steer_target_, AxisInput{io_.steer_pct, io_at_, fresh, io_.steer_limits}, output_.steer, now_ms);
    CheckAxes(now_ms);
    UpdateDrive(now_ms);
    if (faults_ != 0) {
        mode_ = Mode::Fault;
        wait_ = WaitReason::None;
    } else if (wait_ != WaitReason::None) {
        mode_ = Mode::Waiting;
    } else {
        // 零输出并不表示已经保持位置：Stopping/ReverseWait 仍是动作流程的一部分。
        mode_ = bucket_.Status().state == AxisState::Holding && steer_.Status().state == AxisState::Holding ? Mode::Holding : Mode::Tracking;
    }
    return drive_;
}

bool PumpControl::ResetFault(uint32_t now_ms, uint32_t recovered) {
    const uint32_t external = static_cast<uint32_t>(FaultCode::Output) | static_cast<uint32_t>(FaultCode::Mechanism) | static_cast<uint32_t>(FaultCode::Motion);
    if (faults_ == 0 || (faults_ & external & ~recovered) != 0 || !io_.valid || io_.input_fault || io_.mechanism_fault ||
        !io_.bucket_limits.valid || !io_.steer_limits.valid ||
        (io_.bucket_limits.lower && io_.bucket_limits.upper) || (io_.steer_limits.lower && io_.steer_limits.upper) ||
        !Fresh(io_seen_, io_at_, cfg_.io_ms, now_ms) || !bucket_.CanReset(output_.bucket, now_ms) || !steer_.CanReset(output_.steer, now_ms)) {
        return false;
    }
    faults_ = 0;
    bucket_.Reset(now_ms);
    steer_.Reset(now_ms);
    mode_ = Mode::Waiting;
    UpdateDrive(now_ms);
    auth_seen_ = false;
    motion_seen_ = false;
    unmanned_ = false;
    wait_ = WaitReason::Authority;
    return true;
}

Snapshot PumpControl::Status() const {
    Snapshot result;
    result.side = side_;
    result.authority_command = authority_command_;
    result.motion_command = motion_command_;
    result.mode = mode_;
    result.io = io_;
    result.drive = drive_;
    result.bucket_target = bucket_target_;
    result.steer_target = steer_target_;
    result.unmanned = unmanned_;
    result.wait = wait_;
    result.faults = faults_;
    result.fault_at = fault_at_;
    result.auth_age = auth_seen_ ? now_ms_ - auth_at_ : UINT32_MAX;
    result.motion_age = motion_seen_ ? now_ms_ - motion_at_ : UINT32_MAX;
    result.io_age = io_seen_ ? now_ms_ - io_at_ : UINT32_MAX;
    result.bucket = bucket_.Status();
    result.steer = steer_.Status();
    return result;
}

bool PumpControl::Publish(CanPort& port) const {
    return port.Publish(Status());
}

PumpRuntime::PumpRuntime(IoPort& io, CanPort& can) : io_(io), can_(can) {}

bool PumpRuntime::Configure(const Settings& cfg) {
    return Configure(cfg, cfg);
}

bool PumpRuntime::Configure(const Settings& left_config, const Settings& right_config) {
    if (left_config.stop_policy != right_config.stop_policy) return false;
    // 先检查两侧副本，任一配置无效时不留下半套配置。
    PumpControl left_probe = left_;
    PumpControl right_probe = right_;
    if (!left_probe.Configure(left_config) || !right_probe.Configure(right_config)) return false;
    left_ = left_probe;
    right_ = right_probe;
    stop_policy_ = left_config.stop_policy;
    return true;
}

bool PumpRuntime::ReceiveCan(uint32_t id, uint8_t dlc, const uint8_t data[8], uint32_t now_ms) {
    if (!pump_bus_) return false;
    if (left_.ReceiveCan(id, dlc, data, now_ms)) {
        return true;
    }
    return right_.ReceiveCan(id, dlc, data, now_ms);
}

void PumpRuntime::ReadSide(PumpControl& pump, Side side, uint32_t now_ms) {
    OutputSample output;
    if (io_.ReadOutput(side, &output)) pump.ReceiveOutput(output);
    IoSample sample;
    uint32_t sample_at = 0;
    const ReadResult result = io_.Read(side, &sample, &sample_at);
    if (result == ReadResult::Sample) {
        pump.ReceiveIo(sample, sample_at);
    } else if (result != ReadResult::Empty) {
        sample.valid = false;
        pump.ReceiveIo(sample, now_ms);
    }
}

bool PumpRuntime::ApplySide(PumpControl& pump, Side side, uint32_t now_ms) {
    if (io_.Apply(side, pump.Status().drive)) return true;
    pump.ReportFault(FaultCode::Output, now_ms);
    // 再提交一次安全输出；这不能替代远端 IO 的断线保护，也不代表物理输出已经断开。
    io_.Apply(side, pump.Status().drive);
    return false;
}

void PumpRuntime::Cycle(uint32_t now_ms) {
    ReadSide(left_, Side::Left, now_ms);
    ReadSide(right_, Side::Right, now_ms);
    left_.Step(now_ms);
    right_.Step(now_ms);
    if (!pump_bus_) {
        left_.Inhibit(WaitReason::CanBus);
        right_.Inhibit(WaitReason::CanBus);
    }
    if (stop_policy_ == StopPolicy::Together && (left_.Status().faults != 0 || right_.Status().faults != 0)) {
        left_.Inhibit(WaitReason::Peer);
        right_.Inhibit(WaitReason::Peer);
    }
    if (!ApplySide(left_, Side::Left, now_ms) && stop_policy_ == StopPolicy::Together) {
        right_.Inhibit(WaitReason::Peer);
    }
    if (!ApplySide(right_, Side::Right, now_ms) && stop_policy_ == StopPolicy::Together) {
        left_.Inhibit(WaitReason::Peer);
        ApplySide(left_, Side::Left, now_ms);
    }

    if (!publish_seen_ || static_cast<uint32_t>(now_ms - publish_at_) >= 200) {
        left_.Publish(can_);
        right_.Publish(can_);
        publish_at_ = now_ms;
        publish_seen_ = true;
    }
}

void PumpRuntime::SetBus(bool pump_ready, uint32_t now_ms) {
    // 首次不就绪只等待；运行后总线丢失锁存故障，恢复后仍需人工复位和重新授权。
    if (pump_bus_ && !pump_ready) ReportFault(FaultCode::CanBus, now_ms);
    pump_bus_ = pump_ready;
}

Snapshot PumpRuntime::Status(Side side) const {
    return side == Side::Left ? left_.Status() : right_.Status();
}

bool PumpRuntime::ResetFault(Side side, uint32_t now_ms) {
    if (!pump_bus_) return false;
    PumpControl& pump = side == Side::Left ? left_ : right_;
    return pump.ResetFault(now_ms, io_.Recovered(side, pump.Status().faults));
}

void PumpRuntime::ReportFault(FaultCode fault, uint32_t now_ms) {
    left_.ReportFault(fault, now_ms);
    right_.ReportFault(fault, now_ms);
}

} // namespace ind::application::control
