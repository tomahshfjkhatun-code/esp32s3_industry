#pragma once

#include <stdint.h>

namespace ind::hardware::led_ob {

enum class State : uint8_t {
    Off,
    Red,
    Green,
    Yellow
};

// 本模块独占两路 LED GPIO；接口应由同一任务调用。
// 初始化后默认熄灯，重复初始化不会改变当前灯状态。
bool Initialize();

// 熄灯并结束模块使用，保留输出高电平以避免引脚悬空；允许重复调用。
void Deinitialize();

// 初始化后读取两路 GPIO 的实际电平，不缓存颜色状态。
// 未初始化时返回 Off，仅作为默认值，不代表对硬件电平的判断。
State GetState();

// 立即设置灯状态；未初始化、状态无效或 GPIO 写入失败时返回 false。
bool SetState(State state);

} // namespace ind::hardware::led_ob
