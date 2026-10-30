#pragma once

#include <IPAddress.h>
#include <cstdint>
#include <memory>

namespace ind::devices::m31remoteio {

enum class IoResult {
    Ok = 0,         // 本次事务成功；写入仅表示收到正常确认，不代表外部负载反馈。
    InvalidConfig,  // 目标地址或通信参数不满足构造约定。
    InvalidChannel, // 索引越出寄存器保护区，或设备返回非法数据地址异常 0x02。
    InvalidValue,   // AO 写入值不在 0..20000 µA。
    InitFailed,     // 本地通信资源分配或 worker 启动失败；后续调用可重试。
    ConnectFailed,  // 本次 TCP 连接尝试失败。
    Timeout,        // 请求等待超时，不能据此判断通道不存在。
    Disconnected,   // 保留枚举值；当前实现不主动检查网口状态，不返回该项。
    ModbusError     // 其他设备异常、响应校验失败或底层 Modbus 错误。
};

// ESP32-S3 作为 Modbus TCP Client，访问配置为 TCP Server 的 M31 主机。
// 一个对象独占一台主机的 TCP 连接，主机及其扩展模块的同类通道按连续地址访问。
//
// 使用前提：
// - 应用负责初始化 W5500 等网络接口，确认链路及 IPv4 就绪后再执行 IO；
//   本驱动不初始化网络、不查询 ETH 状态，也不能区分本机断网与远端不可达。
// - IO 类型、量程及滤波由上位机配置；增减扩展模块后须完成设备地址协商。
//   驱动不配置量程、不自动识别模块或探测实际 IO 总数。
//
// 生命周期与调用约定：
// - 构造只保存配置，不访问网络或创建任务。首次合法 IO 调用初始化通信，
//   eModbus 按需建连；失败由本次调用返回，后续新调用可再次尝试。
// - 持续使用时复用同一实例；无业务轮询、缓存、心跳或空闲重连任务。
// - 所有调用及析构由同一业务任务串行执行，不能用于 ISR 或网络事件回调。
//   同步 IO 会阻塞当前任务；在 TaskScheduler 回调中也会推迟其他调度任务。
// - 同步调用等待连接、响应及 worker 收尾；两个超时参数均不是调用总耗时
//   上限，不能据此设置业务硬截止时间。
// - 读取仅在返回 Ok 时更新输出参数；失败时保留旧值，使用读数前必须判错。
// - 写请求超时或发送后失联时，设备可能已执行；本驱动不自动补发失败请求。
// - 析构停止通信并释放资源，不主动将 DO 关闭或将 AO 写零。
//
// 通道约定：
// - index 从 0 开始，0 对应同类 IO 的第 1 路，协议地址直接使用 index。
// - AI/AO 索引限制仅保护寄存器用途分区，不保证区内通道实际存在；
//   不存在的通道由设备响应决定，正常读到 0 也不能证明通道存在或不存在。
//
// 依赖基线：Arduino ESP32 3.3.12、eModbus 1.7.6（构建需 Async TCP 3.5.0），
// FreeRTOS tick 必须为 1000 Hz。升级依赖时须复核 .cpp 中 worker 的收尾逻辑。
// 实际 IO 输出、断网恢复时序及板上长期资源稳定性仍需实机验收。
//
// 最小用法（在业务任务中复用 io，网络就绪后执行读取）：
//   M31RemoteIO io(IPAddress(192, 168, 3, 7));
//   bool di = false;
//   if (io.get_di(0, di) == IoResult::Ok) {
//       // 使用本次有效读数 di。
//   }
class M31RemoteIO final {
public:
    // Ip 使用现场主机地址；当前仅接受 IPv4，首字节为 1..223。
    // Port 为 1..65535（默认 502），UnitId 为 1..247（默认 1）。
    // 两个超时均为 1..60000 ms，默认各 1000 ms：ConnectTimeoutMs 交给
    // NetworkClient 设置建连超时，ResponseTimeoutMs 设置 Modbus 响应超时；
    // 均不控制重连间隔或重试次数。参数在 IO 调用时校验，非法返回 InvalidConfig。
    explicit M31RemoteIO(IPAddress Ip, uint16_t Port = 502, uint8_t UnitId = 1,
                         uint32_t ConnectTimeoutMs = 1000,
                         uint32_t ResponseTimeoutMs = 1000);
    ~M31RemoteIO();

    M31RemoteIO(const M31RemoteIO&) = delete;
    M31RemoteIO& operator=(const M31RemoteIO&) = delete;
    M31RemoteIO(M31RemoteIO&&) = delete;
    M31RemoteIO& operator=(M31RemoteIO&&) = delete;

    // DI/DO：index 为 0..65535 的位地址，不按实际硬件通道数做本地校验。
    // 读取数字输入，功能码 0x02，每次读取 1 bit。
    IoResult get_di(uint16_t index, bool& value);

    // 读取数字输出状态，功能码 0x01，每次读取 1 bit。
    IoResult get_do(uint16_t index, bool& value);

    // 设置数字输出，功能码 0x05；true/false 编码为 0xFF00/0x0000。
    IoResult set_do(uint16_t index, bool value);

    // AI：index 为 0..999；1000（0x03E8）起为浮点区，越界返回 InvalidChannel。
    // 两个接口均用功能码 0x04 读取同一整数工程量区，每次 1 个寄存器，
    // 按 16 位有符号数扩展为 int32_t，保留差分负值，不按额定量程裁剪。
    // 调用者按现场通道类型选择接口；驱动不检测类型，也不做电流/电压互换。
    // 电流单位 µA：12500 表示 12.5 mA。
    IoResult get_ai_current(uint16_t index, int32_t& microamps);

    // 电压单位 mV：5000 表示 5 V，-10000 表示 -10 V。
    // 手册 V1.1 第 33 页换算式的单位有误，按工程整数直接表示 mV 解释。
    IoResult get_ai_voltage(uint16_t index, int32_t& millivolts);

    // AO：仅支持电流；index 为 0..499，500（0x01F4）起为过程量区，
    // 越界返回 InvalidChannel。工程量单位 µA，量程由上位机配置。
    // 功能码 0x03 读取 1 个保持寄存器，按无符号数扩展为 int32_t；
    // 读回的是输出工程量寄存器，不是外部实际电流反馈。
    IoResult get_ao_current(uint16_t index, int32_t& microamps);

    // 功能码 0x06 写 1 个保持寄存器；microamps 为 0..20000 µA，
    // 12500 表示请求输出 12.5 mA，越界返回 InvalidValue，不发送请求。
    // 4..20 mA 模式下写入低于 4000 µA 的设备行为须实机核实。
    IoResult set_ao_current(uint16_t index, int32_t microamps);

private:
    struct Impl;

    bool IsValidConfig() const;
    IoResult EnsureInitialized();
    IoResult Exchange(uint8_t function, uint16_t index, uint16_t argument,
                      uint16_t& value);
    IoResult ReadAi(uint16_t index, int32_t& value);

    IPAddress ip_;
    uint16_t port_;
    uint8_t unit_id_;
    uint32_t connect_timeout_ms_;
    uint32_t response_timeout_ms_;
    std::unique_ptr<Impl> impl_;
};

} // namespace ind::devices::m31remoteio
