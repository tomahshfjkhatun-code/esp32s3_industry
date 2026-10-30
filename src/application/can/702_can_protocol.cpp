#include "702_can_protocol.h"

namespace ind::application::can {

bool DecodeCommand(uint32_t pump_id, uint8_t pump_dlc, const uint8_t pump_data[8], Command* pump_command) {
    if (pump_command == nullptr || pump_data == nullptr || pump_dlc != 8) return false;
    Command pump_next;
    pump_next.right_pump = pump_id == 0x04069081U || pump_id == 0x04069082U;
    if (pump_id == 0x04069001U || pump_id == 0x04069081U) {
        pump_next.pump_priority = pump_data[0] & 3U;
        if (pump_next.pump_priority != 1 && pump_next.pump_priority != 2) return false;
        for (uint8_t pump_index = 1; pump_index < 8; ++pump_index) {
            if (pump_data[pump_index] != 0xFF) return false;
        }
    } else if (pump_id == 0x04069002U || pump_id == 0x04069082U) {
        if (pump_data[0] < 27 || pump_data[0] > 227 || pump_data[1] < 27 || pump_data[1] > 227 ||
            pump_data[2] > 100 || (pump_data[4] & 3U) == 3U) return false;
        pump_next.command_kind = CommandKind::Motion;
        pump_next.bucket_target = static_cast<int16_t>(pump_data[0]) - 127;
        pump_next.steer_target = static_cast<int16_t>(pump_data[1]) - 127;
        pump_next.throttle_target = pump_data[2];
        pump_next.clutch_request = (pump_data[3] & 3U) == 1;
        pump_next.backflush_request = ((pump_data[3] >> 2) & 3U) == 1;
        pump_next.engine_action = static_cast<EngineAction>(pump_data[4] & 3U);
        // 备用字段保留 Invalid 状态，不把 3 当成动作，也不否定其它已定义指令。
        pump_next.engine_estop = static_cast<SwitchAction>((pump_data[4] >> 2) & 3U);
        pump_next.engine_warm = static_cast<SwitchAction>((pump_data[4] >> 4) & 3U);
        pump_next.engine_override = static_cast<SwitchAction>((pump_data[4] >> 6) & 3U);
    } else return false;
    *pump_command = pump_next;
    return true;
}

namespace {
template<class T> bool Fresh(const Signal<T>& pump_signal, uint32_t status_ms, uint32_t now_ms) {
    return pump_signal.valid && status_ms != 0 && static_cast<uint32_t>(now_ms - pump_signal.at_ms) <= status_ms;
}
void PutWord(uint8_t* pump_data, uint32_t pump_word) {
    pump_data[0] = static_cast<uint8_t>(pump_word >> 8);
    pump_data[1] = static_cast<uint8_t>(pump_word);
}
bool ValidAlarm(const Alarm& pump_alarm) {
    return pump_alarm.device_type >= 1 && pump_alarm.device_type <= 3 && pump_alarm.alarm_spn <= 0x7FFFFU && pump_alarm.alarm_fmi <= 31;
}
void PutAlarm(uint8_t* pump_data, const Alarm& pump_alarm) {
    pump_data[0] = pump_alarm.device_type | (pump_alarm.alarm_active ? 4U : 0U);
    // SPN 按故障表的明确位定义编码，不能按普通数值的大端顺序排列。
    pump_data[1] = static_cast<uint8_t>(pump_alarm.alarm_spn);
    pump_data[2] = static_cast<uint8_t>(pump_alarm.alarm_spn >> 8);
    pump_data[3] = static_cast<uint8_t>(((pump_alarm.alarm_spn >> 16) << 5) | pump_alarm.alarm_fmi);
}
}

bool ValidRoute(const AlarmRoute& route) {
    return route.ready && route.device_type >= 1 && route.device_type <= 3 && route.spn <= 0x7FFFFU && route.fmi <= 31;
}

bool MapAlarm(const AlarmRoute& route, const Signal<bool>& input, Signal<Alarm>* output) {
    if (output == nullptr) return false;
    *output = {};
    if (!ValidRoute(route) || !input.valid) return false;
    *output = {{route.device_type, input.value == route.active_level, route.spn, route.fmi}, input.at_ms, true};
    return true;
}

EncodeResult EncodeStatus(bool right_pump, uint8_t pump_slot, const PumpStatus& pump_status, const ProtocolSettings& pump_config, uint32_t now_ms, StatusFrame* pump_frame) {
    if (pump_frame == nullptr || pump_slot > 3 || pump_config.status_ms == 0 || pump_config.status_ms > 60000) return EncodeResult::Invalid;
    StatusFrame pump_next;
    const uint32_t pump_offset = right_pump ? 0x80U : 0U;
    pump_next.id = (pump_slot == 3 ? 0x0A069001U : 0x06069001U + pump_slot) + pump_offset;
    for (uint8_t pump_index = 0; pump_index < 8; ++pump_index) pump_next.data[pump_index] = pump_config.reserved_value;
    const uint32_t status_ms = pump_config.status_ms;
    if (pump_slot == 0) {
        if (!pump_config.speed_layout || !pump_config.reserved_layout ||
            !Fresh(pump_status.bucket_angle, status_ms, now_ms) || !Fresh(pump_status.steer_angle, status_ms, now_ms) ||
            !Fresh(pump_status.clutch_engaged, status_ms, now_ms) || !Fresh(pump_status.backflush_active, status_ms, now_ms)) return EncodeResult::NotReady;
        if (pump_status.bucket_angle.value < -32767 || pump_status.bucket_angle.value > 32768 ||
            pump_status.steer_angle.value < -32767 || pump_status.steer_angle.value > 32768) return EncodeResult::Invalid;
        PutWord(pump_next.data, static_cast<uint32_t>(pump_status.bucket_angle.value + 32767));
        PutWord(pump_next.data + 2, static_cast<uint32_t>(pump_status.steer_angle.value + 32767));
        pump_next.data[4] = (pump_config.reserved_value & 0xF0U) | (pump_status.clutch_engaged.value ? 1U : 0U) | (pump_status.backflush_active.value ? 4U : 0U);
        uint32_t engine_speed = 65535;
        if (Fresh(pump_status.engine_speed, status_ms, now_ms)) {
            if (pump_status.engine_speed.value > 65520) return EncodeResult::Invalid;
            engine_speed = pump_status.engine_speed.value;
        }
        pump_next.data[5] = static_cast<uint8_t>(engine_speed >> 16);
        PutWord(pump_next.data + 6, engine_speed);
    } else if (pump_slot == 1) {
        if (!Fresh(pump_status.gearbox_pressure, status_ms, now_ms) || !Fresh(pump_status.gearbox_temperature, status_ms, now_ms) ||
            !Fresh(pump_status.hydraulic_pressure, status_ms, now_ms) || !Fresh(pump_status.gearbox_lubrication, status_ms, now_ms)) return EncodeResult::NotReady;
        if (pump_status.gearbox_pressure.value > 6553500U || pump_status.gearbox_lubrication.value > 6553500U ||
            pump_status.hydraulic_pressure.value > 65535000U || pump_status.gearbox_temperature.value < -32767 ||
            pump_status.gearbox_temperature.value > 32768) return EncodeResult::Invalid;
        PutWord(pump_next.data, pump_status.gearbox_pressure.value / 100U);
        PutWord(pump_next.data + 2, static_cast<uint32_t>(pump_status.gearbox_temperature.value + 32767));
        PutWord(pump_next.data + 4, pump_status.hydraulic_pressure.value / 1000U);
        PutWord(pump_next.data + 6, pump_status.gearbox_lubrication.value / 100U);
    } else if (pump_slot == 2) {
        if (!pump_config.reserved_layout || !Fresh(pump_status.valve_position, status_ms, now_ms) ||
            !Fresh(pump_status.hydraulic_running, status_ms, now_ms)) return EncodeResult::NotReady;
        if (pump_status.valve_position.value < -100 || pump_status.valve_position.value > 100) return EncodeResult::Invalid;
        pump_next.data[1] = static_cast<uint8_t>(pump_status.valve_position.value + 100);
        pump_next.data[2] = (pump_config.reserved_value & 0xFCU) | (pump_status.hydraulic_running.value ? 1U : 2U);
    } else {
        if (!pump_config.reserved_layout || !Fresh(pump_status.pump_alarms[0], status_ms, now_ms) ||
            !Fresh(pump_status.pump_alarms[1], status_ms, now_ms)) return EncodeResult::NotReady;
        if (!ValidAlarm(pump_status.pump_alarms[0].value) || !ValidAlarm(pump_status.pump_alarms[1].value)) return EncodeResult::Invalid;
        PutAlarm(pump_next.data, pump_status.pump_alarms[0].value);
        PutAlarm(pump_next.data + 4, pump_status.pump_alarms[1].value);
        pump_next.data[0] |= pump_config.reserved_value & 0xF0U;
        pump_next.data[4] |= pump_config.reserved_value & 0xF0U;
    }
    *pump_frame = pump_next;
    return EncodeResult::Ready;
}

} // namespace ind::application::can
