#pragma once

#include <stdint.h>
#include "axis_control.h"
#include "../can/702_can_protocol.h"

namespace ind::application::control {

enum class Side : uint8_t { Left, Right };
enum class Mode : uint8_t { Waiting, Tracking, Holding, Fault };
enum class WaitReason : uint8_t { None, Configuration, Authority, Motion, Input, Peer, CanBus };
enum class FaultCode : uint32_t { Input = 1U, Output = 2U, RxOverflow = 4U, Mechanism = 8U, Motion = 16U, CanBus = 32U };
enum class ReadResult : uint8_t { Empty, Sample, Fault };
enum class StopPolicy : uint8_t { Unspecified, Independent, Together };

// 位置由 702 IO 适配层标定为 -100..100；这里不使用原始 4-20mA 值。
struct IoSample {
    int16_t bucket_pct = 0;
    int16_t steer_pct = 0;
    int16_t valve_pct = 0;
    bool valve_valid = false;
    bool valid = false;
    bool input_fault = false;
    bool mechanism_fault = false;
    AxisLimits bucket_limits;
    AxisLimits steer_limits;
};

struct OutputSample {
    AxisAck bucket;
    AxisAck steer;
};

struct Drive {
    // -1=下降/左转，0=请求安全输出，1=上升/右转；具体 DO/PWM 映射由 IO 实现方确定。
    int8_t bucket_dir = 0;
    int8_t steer_dir = 0;
    bool enabled = false;
    uint16_t bucket_effort = 0;
    uint16_t steer_effort = 0;
    uint32_t bucket_seq = 0;
    uint32_t steer_seq = 0;
    uint32_t issued_at = 0;
    uint32_t lease_ms = 0;
};

struct Settings {
    uint32_t auth_ms = 0;
    uint32_t motion_ms = 0;
    uint32_t io_ms = 0;
    AxisSettings bucket;
    AxisSettings steer;
    StopPolicy stop_policy = StopPolicy::Unspecified;
};

struct Snapshot {
    Side side = Side::Left;
    Mode mode = Mode::Waiting;
    IoSample io;
    Drive drive;
    int16_t bucket_target = 0;
    int16_t steer_target = 0;
    bool unmanned = false;
    WaitReason wait = WaitReason::Configuration;
    uint32_t faults = 0;
    uint32_t fault_at = 0;
    uint32_t auth_age = UINT32_MAX;
    uint32_t motion_age = UINT32_MAX;
    uint32_t io_age = UINT32_MAX;
    can::CommandStatus authority_command;
    can::CommandStatus motion_command;
    AxisStatus bucket;
    AxisStatus steer;
};

// Io702 在此接入同事的 M31 Modbus TCP：ESP32 为客户端，主机出厂地址 192.168.3.7:502。
// Unit ID 暂按出厂 Modbus 地址 1 交接；具体 TCP 行为、通道和量程须按实物手册核对。
// Read：将 AI 标定为 -100..100，填入有效性、原始采样时间及外部限位证据，禁止伪造有效采样。
// Read/Apply/Recovered 必须操作缓存或非阻塞邮箱，不在控制任务等待网络事务。
// Empty 不刷新采样时间；Fault 是确认异常。Apply=false 仅表示提交失败，不能证明线圈已断电。
class IoPort {
public:
    virtual ~IoPort() = default;
    virtual ReadResult Read(Side side, IoSample* sample, uint32_t* sample_at) = 0;
    // Io702 保证倒斗两个 DO 的请求互斥；转舵 direction+effort 转为 PWM 占空比。
    // 合并左右机最新完整输出；停止覆盖旧动作。按 issued_at+lease_ms 使过期请求失效。
    // enabled=false 时仍须执行安全输出，不能忽略；安全电平/液压状态必须由 IO 同事明确。
    virtual bool Apply(Side side, const Drive& drive) = 0;
    // IoHooks 返回现场约定的输出执行反馈，保留请求序号和反馈采集时间，不能把提交成功直接作确认。
    // 缺省无反馈：配置动作参数后仍不能通过初始安全输出确认，不会启动运动。
    virtual bool ReadOutput(Side, OutputSample*) { return false; }
    // 返回已有恢复证据的故障位。Output 必须确认输出通道可用且安全输出已执行；
    // Mechanism/Motion 必须确认相应故障原因已排除。仅输入恢复不构成这些证据。
    virtual uint32_t Recovered(Side, uint32_t) { return 0; }
};

// 发布层按协议的 200ms 周期调用 Publish，实现状态帧编码与 CanService::Send；
// 字段不全时返回 false，不发伪数据。
class CanPort {
public:
    virtual ~CanPort() = default;
    virtual bool Publish(const Snapshot& status) = 0;
};

// 所有方法由同一个控制任务调用。CAN RX 回调应先把报文复制到事件队列；
// Modbus 采集任务也应通过队列交付 IoSample，避免跨任务直接访问状态机。
// ReceiveCan/ReceiveIo 的 now_ms 使用接收/采样时刻；Step/ResetFault 使用当前时刻。
// 全部使用同一单调毫秒时钟，报文不能用出队时间代替接收时间。
class PumpControl final {
public:
    explicit PumpControl(Side side);

    bool Configure(const Settings& cfg);
    static bool ValidCan(uint32_t id, uint8_t dlc, const uint8_t data[8]);
    bool ReceiveCan(uint32_t id, uint8_t dlc, const uint8_t data[8], uint32_t now_ms);
    void ReceiveIo(const IoSample& sample, uint32_t now_ms);
    void ReceiveOutput(const OutputSample& sample);
    void ReportFault(FaultCode fault, uint32_t now_ms);
    void Inhibit(WaitReason reason);
    Drive Step(uint32_t now_ms);
    bool ResetFault(uint32_t now_ms, uint32_t recovered);
    Snapshot Status() const;
    bool Publish(CanPort& port) const;

private:
    bool Fresh(bool seen, uint32_t stamp, uint32_t limit, uint32_t now_ms) const;
    void Expire(uint32_t now_ms);
    void UpdateDrive(uint32_t now_ms);
    void CheckAxes(uint32_t now_ms);

    Side side_;
    Mode mode_ = Mode::Waiting;
    can::CommandStatus authority_command_;
    can::CommandStatus motion_command_;
    Settings cfg_;
    IoSample io_;
    Drive drive_;
    OutputSample output_;
    AxisControl bucket_;
    AxisControl steer_;
    uint32_t auth_at_ = 0;
    uint32_t motion_at_ = 0;
    uint32_t io_at_ = 0;
    int16_t bucket_target_ = 0;
    int16_t steer_target_ = 0;
    bool configured_ = false;
    bool auth_seen_ = false;
    bool motion_seen_ = false;
    bool io_seen_ = false;
    bool unmanned_ = false;
    uint32_t faults_ = 0;
    uint32_t fault_at_ = 0;
    uint32_t now_ms_ = 0;
    WaitReason wait_ = WaitReason::Configuration;
};

// 控制任务从事件队列取 CAN 帧并调用 ReceiveCan，再周期调用 Cycle。
// 本类不创建任务，也不在 CAN RX 回调里执行 Modbus 操作。
class PumpRuntime final {
public:
    PumpRuntime(IoPort& io, CanPort& can);

    bool Configure(const Settings& cfg);
    bool Configure(const Settings& left_config, const Settings& right_config);
    bool ReceiveCan(uint32_t id, uint8_t dlc, const uint8_t data[8], uint32_t now_ms);
    void Cycle(uint32_t now_ms);
    void SetBus(bool pump_ready, uint32_t now_ms);
    Snapshot Status(Side side) const;
    bool ResetFault(Side side, uint32_t now_ms);
    void ReportFault(FaultCode fault, uint32_t now_ms);

private:
    void ReadSide(PumpControl& pump, Side side, uint32_t now_ms);
    bool ApplySide(PumpControl& pump, Side side, uint32_t now_ms);

    bool pump_bus_ = false;
    IoPort& io_;
    CanPort& can_;
    PumpControl left_{Side::Left};
    PumpControl right_{Side::Right};
    uint32_t publish_at_ = 0;
    bool publish_seen_ = false;
    StopPolicy stop_policy_ = StopPolicy::Unspecified;
};

} // namespace ind::application::control
