#include "led_ob.h"

#include <driver/gpio.h>

namespace ind::hardware::led_ob {
namespace {

constexpr gpio_num_t led_pb_red = GPIO_NUM_35;
constexpr gpio_num_t led_pb_green = GPIO_NUM_42;

// 板载双色 LED 低电平点亮，两路同时点亮为黄色。
constexpr uint32_t level_on = 0;
constexpr uint32_t level_off = 1;

bool initialized = false;

bool WriteLevels(uint32_t red_level, uint32_t green_level)
{
    // 两路均尝试写入，避免第一路失败时跳过第二路。
    const esp_err_t red_result = gpio_set_level(led_pb_red, red_level);
    const esp_err_t green_result = gpio_set_level(led_pb_green, green_level);
    return red_result == ESP_OK && green_result == ESP_OK;
}

} // 匿名命名空间

bool Initialize()
{
    if (initialized)
    {
        return true;
    }

    // 先预置输出高电平，再启用输出，避免初始化时短暂点亮。
    if (!WriteLevels(level_off, level_off))
    {
        return false;
    }

    gpio_config_t config{};
    config.pin_bit_mask = (1ULL << led_pb_red) | (1ULL << led_pb_green);
    // 同时开启输入通路，确保 GetState() 能读到引脚电平。
    config.mode = GPIO_MODE_INPUT_OUTPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;

    if (gpio_config(&config) != ESP_OK)
    {
        return false;
    }

    initialized = true;
    return true;
}

void Deinitialize()
{
    if (!initialized)
    {
        return;
    }

    // 保留 GPIO 输出配置，持续拉高以保持熄灯。
    // 若写入失败，保留初始化标记，允许再次调用释放接口重试。
    if (WriteLevels(level_off, level_off))
    {
        initialized = false;
    }
}

State GetState()
{
    if (!initialized)
    {
        return State::Off;
    }

    const bool red_on = gpio_get_level(led_pb_red) == level_on;
    const bool green_on = gpio_get_level(led_pb_green) == level_on;

    if (red_on && green_on)
    {
        return State::Yellow;
    }
    if (red_on)
    {
        return State::Red;
    }
    if (green_on)
    {
        return State::Green;
    }
    return State::Off;
}

bool SetState(State state)
{
    if (!initialized)
    {
        return false;
    }

    switch (state)
    {
    case State::Off:
        return WriteLevels(level_off, level_off);
    case State::Red:
        return WriteLevels(level_on, level_off);
    case State::Green:
        return WriteLevels(level_off, level_on);
    case State::Yellow:
        return WriteLevels(level_on, level_on);
    default:
        return false;
    }
}

} // namespace ind::hardware::led_ob
