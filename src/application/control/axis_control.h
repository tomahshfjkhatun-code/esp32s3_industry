#pragma once

#include <stdint.h>

namespace ind::application::control {

enum class AxisState : uint8_t { Disabled, Holding, Starting, Moving, Stopping, ReverseWait, Fault };
enum class AxisFault : uint32_t { Input = 1U, Limit = 2U, StartTimeout = 4U, StopTimeout = 8U, FeedbackLost = 16U, TravelTimeout = 32U, NoProgress = 64U, WrongDirection = 128U };

// 本阶段沿用 CAN 的整数百分比单位，范围 -100..100；不是角度或 AI 电流原始值。
// 参数全零表示尚未配置，禁止动作。数值必须依据执行器、传感器和 IO 延迟确定。
//这些值目前是待依据执行器、位置传感器和 IO 通信延迟确定的配置参数，不能仅凭代码中的允许范围当作现场设定值
struct AxisSettings {
    int16_t min_pct = 0;//允许到达的最小位置	目标或采集到的位置低于它，报限位故障
    int16_t max_pct = 0;//允许到达的最大位置	目标或采集到的位置高于它，报限位故障
    uint8_t start_tol = 0;//偏离目标多少才开始动	保持状态下，误差超过它才请求启动
    uint8_t stop_tol = 0;//离目标多近就停止	运动中，误差进入这个范围就请求停止
    uint8_t min_progress = 0;//多大位移才算“确实在向目标前进”	达到后更新位移监测的起点和时间
    uint8_t wrong_tol = 0;//反方向移动多少算异常	达到后报反向位移故障
    uint16_t effort = 0;//动作时请求的驱动力大小，范围 1～1000	发出动作请求时使用；具体怎样转换成 IO 输出由 IO 层决定
    uint32_t ack_ms = 0;//等待并确认输出已按请求生效的最长时间	启动或停止得不到有效执行反馈时，报超时故障；也用于判断反馈是否过期
    uint32_t travel_ms = 0;//一次连续动作允许的最长时间	从请求启动开始计时，包含启动确认阶段；超时仍未结束就报故障
    uint32_t stall_ms = 0;//一直没有达到最小位移进展时，最多等多久	超时后报“无足够位移”故障；它本身不能证明机械堵转
    uint32_t grace_ms = 0;//刚开始运动后，暂时不检查位移进展的宽限时间	避免把启动前或刚启动时的旧传感器数据当成运动结果
    uint32_t reverse_ms = 0;//停止输出已确认后，换向前要等待多久	等待期结束，才回到可发起新动作的保持状态
};

struct AxisLimits {
    bool valid = false;
    bool lower = false;
    bool upper = false;
};

struct AxisInput {
    int16_t position = 0;
    uint32_t at_ms = 0;
    bool valid = false;
    AxisLimits limits;
};

struct AxisDemand {
    int8_t direction = 0;
    // 0..1000 的归一化驱动力请求；IO 层负责转换并限幅，不能直接当作 PWM 占空比。
    uint16_t effort = 0;
    uint32_t sequence = 0;
};

struct AxisAck {
    AxisDemand applied;
    uint32_t at_ms = 0;
    // 由 IO 实现方依据已约定的输出反馈提供；仅入队、Modbus 应答或寄存器回读不够。
    // direction=0 的有效反馈表示约定的安全输出已生效，不宣称机械部件完全静止。
    bool valid = false;
};

struct AxisStatus {
    AxisState state = AxisState::Disabled;
    uint32_t faults = 0;
    AxisDemand demand;
};

// 无任务、无等待、无网络操作。四个实例分别用于左右机的倒斗和转舵。
class AxisControl final {
public:
    bool Configure(const AxisSettings& cfg);
    bool Ready() const;
    AxisStatus Step(bool allow, int16_t target, const AxisInput& input, const AxisAck& ack, uint32_t now_ms);
    void Stop(uint32_t now_ms);
    bool CanReset(const AxisAck& ack, uint32_t now_ms) const;
    void Reset(uint32_t now_ms);
    AxisStatus Status() const;

private:
    static bool Same(const AxisSettings& lhs, const AxisSettings& rhs);
    static int8_t Direction(int16_t target, int16_t actual, uint8_t tolerance);
    bool Confirmed(const AxisAck& ack, uint32_t now_ms) const;
    void Demand(int8_t direction, uint32_t now_ms);
    void Trip(AxisFault fault, uint32_t now_ms);

    AxisSettings cfg_;
    AxisStatus status_;
    bool ready_ = false;
    bool monitoring_ = false;
    uint32_t changed_at_ = 0;
    uint32_t travel_at_ = 0;
    uint32_t moving_at_ = 0;
    uint32_t neutral_at_ = 0;
    uint32_t progress_at_ = 0;
    uint32_t sample_at_ = 0;
    int16_t anchor_ = 0;
};

} // namespace ind::application::control
