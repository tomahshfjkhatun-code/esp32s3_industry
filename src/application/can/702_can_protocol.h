#pragma once

#include <stdint.h>

// PJ400无人控制通信协议_20260903；与驱动和 RTOS 无关。
namespace ind::application::can {

enum class CommandKind : uint8_t { Authority, Motion };
enum class EngineAction : uint8_t { None, Start, Stop, Invalid };
enum class SwitchAction : uint8_t { None, On, Off, Invalid };
enum class EncodeResult : uint8_t { Ready, NotReady, Invalid };

struct Command {
    bool right_pump = false;
    CommandKind command_kind = CommandKind::Authority;
    uint8_t pump_priority = 0;
    int16_t bucket_target = 0;
    int16_t steer_target = 0;
    uint8_t throttle_target = 0;
    bool clutch_request = false;
    bool backflush_request = false;
    EngineAction engine_action = EngineAction::None;
    SwitchAction engine_estop = SwitchAction::None;
    SwitchAction engine_warm = SwitchAction::None;
    SwitchAction engine_override = SwitchAction::None;
};

struct CommandStatus {
    Command pump_command;
    uint32_t command_at = 0;
    bool command_seen = false;
    bool command_accepted = false;
};

template<class T> struct Signal {
    T value{};
    uint32_t at_ms = 0;
    bool valid = false;
};

struct Alarm {
    uint8_t device_type = 0;
    bool alarm_active = false;
    uint32_t alarm_spn = 0;
    uint8_t alarm_fmi = 0;
};

struct AlarmRoute {
    bool ready = false;
    uint8_t device_type = 0;
    uint32_t spn = 0;
    uint8_t fmi = 0;
    bool active_level = true;
};

struct PumpStatus {
    Signal<int32_t> bucket_angle;
    Signal<int32_t> steer_angle;
    Signal<bool> clutch_engaged;
    Signal<bool> backflush_active;
    Signal<uint32_t> engine_speed;
    Signal<uint32_t> gearbox_pressure;
    Signal<int32_t> gearbox_temperature;
    Signal<uint32_t> hydraulic_pressure;
    Signal<uint32_t> gearbox_lubrication;
    Signal<int16_t> valve_position;
    Signal<bool> hydraulic_running;
    Signal<Alarm> pump_alarms[2];
};

// angle=0.01度，temperature=0.1度，pressure=Pa，speed=0.125rpm。
// 速度字段文档占3字节而量程只有16位；确认其24位大端编码前不发送位置帧。
struct ProtocolSettings {
    uint32_t status_ms = 1000;
    bool speed_layout = false;
    bool reserved_layout = false;
    uint8_t reserved_value = 0xFF;
};

struct StatusFrame {
    uint32_t id = 0;
    uint8_t dlc = 8;
    uint8_t data[8]{};
};

bool DecodeCommand(uint32_t pump_id, uint8_t pump_dlc, const uint8_t pump_data[8], Command* pump_command);
bool ValidRoute(const AlarmRoute& route);
bool MapAlarm(const AlarmRoute& route, const Signal<bool>& input, Signal<Alarm>* output);
EncodeResult EncodeStatus(bool right_pump, uint8_t pump_slot, const PumpStatus& pump_status, const ProtocolSettings& pump_config, uint32_t now_ms, StatusFrame* pump_frame);

} // namespace ind::application::can
