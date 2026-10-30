#pragma once

#include <stdint.h>

// 702 逻辑信号到 M31 零基索引及板载 GPIO 的集中映射。
// 左侧按现场表 AI1..AI4；右侧暂按相同顺序分配 AI5..AI8。
// 修改接线时先核对设备地址，再只调整这里的映射。
// PWM 只预留引脚；频率、占空比、极性、方向及力度换算由后续驱动接口配置。
namespace ind::application::io::map {

namespace di {
inline constexpr uint16_t main_oil_low = 0;
inline constexpr uint16_t main_oil_hot = 1;
inline constexpr uint16_t left_filter_alarm = 2;
inline constexpr uint16_t right_filter_alarm = 3;
}

namespace ai {
inline constexpr uint16_t left_valve_pos = 0;
inline constexpr uint16_t left_bucket_pos = 1;
inline constexpr uint16_t left_steer_pos = 2;
inline constexpr uint16_t left_pressure = 3;
inline constexpr uint16_t right_valve_pos = 4;
inline constexpr uint16_t right_bucket_pos = 5;
inline constexpr uint16_t right_steer_pos = 6;
inline constexpr uint16_t right_pressure = 7;
}

namespace dout {
inline constexpr uint16_t left_bucket_up = 0;
inline constexpr uint16_t left_bucket_down = 1;
inline constexpr uint16_t right_bucket_up = 2;
inline constexpr uint16_t right_bucket_down = 3;
}

namespace pwm {
inline constexpr uint8_t left_steer_gpio = 4;
inline constexpr uint8_t right_steer_gpio = 5;
}

} // namespace ind::application::io::map
