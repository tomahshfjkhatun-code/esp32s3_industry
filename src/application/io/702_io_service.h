#pragma once

#include <IPAddress.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../../devices/m31remoteio/m31remoteio.h"
#include "702_pwm.h"

namespace ind::application::io {

using IoResult = devices::m31remoteio::IoResult;

struct NetworkConfig {
    IPAddress m31_address{192, 168, 3, 7};
    uint16_t m31_port = 502;
    uint8_t m31_unit = 1;
    uint32_t connect_ms = 1000;
    uint32_t response_ms = 1000;
};

template<class T> struct IoValue {
    T value{};
    uint32_t at_ms = 0;
    uint32_t attempt_at = 0;
    IoResult result = IoResult::Disconnected;
    bool valid = false;
};

struct WriteStatus {
    control::Drive pump_request;
    IoResult write_result = IoResult::Disconnected;
    uint32_t write_at = 0;
    bool write_fault = false;
    bool write_complete = false;
    bool write_uncertain = false;
};

struct PumpIo {
    IoValue<int32_t> valve_current;
    IoValue<int32_t> bucket_current;
    IoValue<int32_t> steer_current;
    IoValue<int32_t> pressure_current;
    IoValue<bool> filter_alarm;
    IoValue<bool> bucket_up;
    IoValue<bool> bucket_down;
    can::Signal<int16_t> bucket_position;
    can::Signal<int16_t> steer_position;
    can::Signal<int16_t> valve_position;
    can::Signal<int32_t> bucket_angle;
    can::Signal<int32_t> steer_angle;
    can::Signal<uint32_t> hydraulic_pressure;
    control::AxisLimits bucket_limits;
    control::AxisLimits steer_limits;
    uint32_t limits_at = 0;
    control::OutputSample pump_feedback;
    WriteStatus pump_write;
    PwmStatus steer_pwm;
    uint32_t recovered_faults = 0;
};

struct IoStatus {
    bool network_ready = false;
    IoValue<bool> main_oil_low;
    IoValue<bool> main_oil_hot;
    PumpIo left_pump;
    PumpIo right_pump;
};

// 钩子在唯一 M31 业务任务中调用。只能读取其它模块缓存，不能阻塞。
// 没有实体限位/执行反馈通道时保持空函数；禁止用 DO 回读伪造 AxisAck。
struct IoHooks {
    bool (*pump_limits)(control::Side pump_side, control::AxisLimits* bucket_limits, control::AxisLimits* steer_limits, uint32_t* limits_at) = nullptr;
    bool (*pump_feedback)(control::Side pump_side, const control::Drive& pump_request, control::OutputSample* pump_feedback) = nullptr;
    uint32_t (*pump_recovered)(control::Side pump_side) = nullptr;
};

class Io702 final : public control::IoPort {
public:
    bool Start(const NetworkConfig& m31_config, const IoConfig& pump_config, const IoHooks& pump_hooks);
    IoStatus get_702_io() const;
    bool set_702_io(control::Side pump_side, const control::Drive& pump_drive);
    control::ReadResult Read(control::Side pump_side, control::IoSample* pump_sample, uint32_t* sample_at) override;
    bool Apply(control::Side pump_side, const control::Drive& pump_drive) override;
    bool ReadOutput(control::Side pump_side, control::OutputSample* pump_feedback) override;
    uint32_t Recovered(control::Side pump_side, uint32_t pump_faults) override;

private:
    static void Run(void* pump_context);
    void Poll(devices::m31remoteio::M31RemoteIO& m31_device, uint8_t channel_index);
    void Write(devices::m31remoteio::M31RemoteIO& m31_device, control::Side pump_side);
    void Hooks(control::Side pump_side);
    void Convert(PumpIo& pump_status, const PumpConfig& pump_config, uint32_t now_ms) const;
    control::Drive Request(control::Side pump_side) const;
    PumpIo& Cache(control::Side pump_side);
    const PumpConfig& Config(control::Side pump_side) const;

    mutable portMUX_TYPE io_lock_ = portMUX_INITIALIZER_UNLOCKED;
    IoStatus io_status_;
    control::Drive left_drive_;
    control::Drive right_drive_;
    OutputSequence left_write_;
    OutputSequence right_write_;
    NetworkConfig m31_config_;
    IoConfig pump_config_;
    IoHooks pump_hooks_;
    Pwm702 steer_pwm_;
    TaskHandle_t io_task_ = nullptr;
};

} // namespace ind::application::io
