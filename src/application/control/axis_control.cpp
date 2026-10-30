#include "axis_control.h"

namespace ind::application::control {

bool AxisControl::Same(const AxisSettings& lhs, const AxisSettings& rhs) {
    return lhs.min_pct == rhs.min_pct && lhs.max_pct == rhs.max_pct && lhs.start_tol == rhs.start_tol &&
           lhs.stop_tol == rhs.stop_tol && lhs.min_progress == rhs.min_progress && lhs.wrong_tol == rhs.wrong_tol &&
           lhs.effort == rhs.effort && lhs.ack_ms == rhs.ack_ms && lhs.travel_ms == rhs.travel_ms &&
           lhs.stall_ms == rhs.stall_ms && lhs.grace_ms == rhs.grace_ms && lhs.reverse_ms == rhs.reverse_ms;
}

bool AxisControl::Configure(const AxisSettings& cfg) {
    if (ready_) return Same(cfg_, cfg);
    if (Same(cfg, AxisSettings{})) return true;
    if (cfg.min_pct < -100 || cfg.max_pct > 100 || cfg.min_pct >= cfg.max_pct ||
        cfg.start_tol <= cfg.stop_tol || cfg.start_tol > 100 || cfg.min_progress == 0 || cfg.min_progress > 200 ||
        cfg.wrong_tol == 0 || cfg.wrong_tol > 200 || cfg.effort == 0 || cfg.effort > 1000 ||
        cfg.ack_ms == 0 || cfg.ack_ms > 60000 || cfg.travel_ms == 0 || cfg.travel_ms > 60000 ||
        cfg.stall_ms == 0 || cfg.stall_ms > 60000 || cfg.grace_ms > 60000 ||
        cfg.reverse_ms == 0 || cfg.reverse_ms > 60000 ||
        cfg.ack_ms + cfg.grace_ms + cfg.stall_ms >= cfg.travel_ms) return false;
    cfg_ = cfg;
    ready_ = true;
    return true;
}

bool AxisControl::Ready() const { return ready_; }
//tolerance（允许的误差范围）根据位置差的正负来决定向哪个方向走
int8_t AxisControl::Direction(int16_t target, int16_t actual, uint8_t tolerance) {
    const int16_t error = target - actual;
    if (error > tolerance) return 1;
    if (error < -static_cast<int16_t>(tolerance)) return -1;
    return 0;
}
//判断：**IO 返回的执行反馈 ack，能否证明“当前这一次输出请求”已经生效。**只有下面所有条件都满足，才返回 true
bool AxisControl::Confirmed(const AxisAck& ack, uint32_t now_ms) const {
    return ack.valid && ack.applied.sequence == status_.demand.sequence &&
           ack.applied.direction == status_.demand.direction && ack.applied.effort == status_.demand.effort &&
           static_cast<uint32_t>(now_ms - ack.at_ms) <= cfg_.ack_ms &&
           static_cast<uint32_t>(ack.at_ms - changed_at_) < 0x80000000U;
}
//例如调用 Demand(1, 1000)，就是在时间 1000 ms 记录一条“正方向、配置力度”的新请求，并更新序号。
//状态机后续通过 IO 接口提交这个请求；Demand() 本身不发送 Modbus 命令
void AxisControl::Demand(int8_t direction, uint32_t now_ms) {
    status_.demand.direction = direction;
    status_.demand.effort = direction == 0 ? 0 : cfg_.effort;
    ++status_.demand.sequence;
    if (status_.demand.sequence == 0) ++status_.demand.sequence;
    changed_at_ = now_ms;
}

void AxisControl::Stop(uint32_t now_ms) {
    if (!ready_) return;
    if (status_.demand.direction != 0 || status_.demand.sequence == 0) {
        Demand(0, now_ms);
        status_.state = status_.faults == 0 ? AxisState::Stopping : AxisState::Fault;
        monitoring_ = false;
    }
}
//Trip() 是状态机统一的故障入口。当检测到限位、启动超时、无位移等异常时，都通过它执行同一套处理
void AxisControl::Trip(AxisFault fault, uint32_t now_ms) {
    status_.faults |= static_cast<uint32_t>(fault);//// 记下故障，保留已有故障
    Stop(now_ms);//// 把该轴的输出请求改为停止
    status_.state = AxisState::Fault;// 锁定故障状态
}

AxisStatus AxisControl::Step(bool allow, int16_t target, const AxisInput& input, const AxisAck& ack, uint32_t now_ms) {
    if (!ready_) return status_;
    if (!allow) Stop(now_ms);
    if (allow && (!input.valid || !input.limits.valid || (input.limits.lower && input.limits.upper))) {
        Trip(AxisFault::Input, now_ms);
    }
    if (allow && (target < cfg_.min_pct || target > cfg_.max_pct || input.position < cfg_.min_pct || input.position > cfg_.max_pct)) {
        Trip(AxisFault::Limit, now_ms);
    }
    const int8_t direction = Direction(target, input.position, cfg_.stop_tol);
    if (allow && ((direction < 0 && input.limits.lower) || (direction > 0 && input.limits.upper))) {
        Trip(AxisFault::Limit, now_ms);
    }
    // 同方向周期指令不刷新 travel_at_；启动确认等待也计入连续动作期限。
    if ((status_.state == AxisState::Starting || status_.state == AxisState::Moving) &&
        static_cast<uint32_t>(now_ms - travel_at_) >= cfg_.travel_ms) Trip(AxisFault::TravelTimeout, now_ms);

    switch (status_.state) {
    case AxisState::Disabled:
        Stop(now_ms);
        break;
    case AxisState::Holding:
        if (!Confirmed(ack, now_ms)) {
            Trip(AxisFault::FeedbackLost, now_ms);
        } else if (allow && Direction(target, input.position, cfg_.start_tol) != 0) {
            Demand(direction, now_ms);
            travel_at_ = now_ms;
            status_.state = AxisState::Starting;
        }
        break;
    case AxisState::Starting:
        if (!allow || direction != status_.demand.direction) {
            Stop(now_ms);
        } else if (Confirmed(ack, now_ms) && static_cast<uint32_t>(ack.at_ms - changed_at_) <= cfg_.ack_ms) {
            moving_at_ = now_ms;
            monitoring_ = false;
            status_.state = AxisState::Moving;
        } else if (static_cast<uint32_t>(now_ms - changed_at_) >= cfg_.ack_ms) {
            Trip(AxisFault::StartTimeout, now_ms);
        }
        break;
    case AxisState::Moving:
        if (!allow || direction != status_.demand.direction) {
            Stop(now_ms);
            break;
        }
        if (!Confirmed(ack, now_ms)) {
            Trip(AxisFault::FeedbackLost, now_ms);
            break;
        }
        if (static_cast<uint32_t>(now_ms - moving_at_) < cfg_.grace_ms) break;
        // 启动前或启动宽限期内的旧位置不能作为动作后的位移证据。
        if (static_cast<uint32_t>(input.at_ms - moving_at_) >= 0x80000000U ||
            static_cast<uint32_t>(input.at_ms - moving_at_) < cfg_.grace_ms) break;
        if (!monitoring_) {
            monitoring_ = true;
            anchor_ = input.position;
            sample_at_ = input.at_ms;
            progress_at_ = now_ms;
        } else if (input.at_ms != sample_at_ && static_cast<uint32_t>(input.at_ms - sample_at_) < 0x80000000U) {
            sample_at_ = input.at_ms;
            const int16_t progress = (input.position - anchor_) * status_.demand.direction;
            if (progress <= -static_cast<int16_t>(cfg_.wrong_tol)) {
                Trip(AxisFault::WrongDirection, now_ms);
            } else if (progress >= cfg_.min_progress) {
                anchor_ = input.position;
                progress_at_ = now_ms;
            } else if (static_cast<uint32_t>(now_ms - progress_at_) >= cfg_.stall_ms) {
                // 仅说明没有检测到足够的位移，不能直接断言机械堵转。
                Trip(AxisFault::NoProgress, now_ms);
            }
        }
        break;
    case AxisState::Stopping:
        if (Confirmed(ack, now_ms) && static_cast<uint32_t>(ack.at_ms - changed_at_) <= cfg_.ack_ms) {
            neutral_at_ = now_ms;
            status_.state = AxisState::ReverseWait;
        } else if (static_cast<uint32_t>(now_ms - changed_at_) >= cfg_.ack_ms) {
            Trip(AxisFault::StopTimeout, now_ms);
        }
        break;
    case AxisState::ReverseWait:
        if (!Confirmed(ack, now_ms)) {
            Trip(AxisFault::FeedbackLost, now_ms);
        } else if (static_cast<uint32_t>(now_ms - neutral_at_) >= cfg_.reverse_ms) {
            status_.state = AxisState::Holding;
        }
        break;
    case AxisState::Fault:
        // 故障仍监视停止反馈，避免最初的动作故障掩盖后续输出未断开。
        if (!Confirmed(ack, now_ms) && static_cast<uint32_t>(now_ms - changed_at_) >= cfg_.ack_ms) {
            status_.faults |= static_cast<uint32_t>(AxisFault::StopTimeout);
        }
        break;
    default:
        Trip(AxisFault::Input, now_ms);
        break;
    }
    return status_;
}

bool AxisControl::CanReset(const AxisAck& ack, uint32_t now_ms) const {
    return !ready_ || (status_.demand.direction == 0 && Confirmed(ack, now_ms));
}

void AxisControl::Reset(uint32_t now_ms) {
    // 只能由控制任务在两个轴的 CanReset 及整机复位条件全部满足后调用。
    status_.faults = 0;
    status_.state = ready_ ? AxisState::ReverseWait : AxisState::Disabled;
    neutral_at_ = now_ms;
    monitoring_ = false;
}

AxisStatus AxisControl::Status() const { return status_; }

} // namespace ind::application::control
