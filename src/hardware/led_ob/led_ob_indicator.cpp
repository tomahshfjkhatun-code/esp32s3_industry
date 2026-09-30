#include "led_ob_indicator.h"

namespace ind::hardware::led_ob {
namespace {

namespace led = ::ind::hardware::led_ob;

constexpr uint32_t max_duration_ms = 0x7fffffffU;
constexpr uint32_t burst_period_ms = 1000;
constexpr uint32_t burst_on_ms = 100;
constexpr uint32_t burst_gap_ms = 100;
constexpr uint8_t max_burst_count = 3;

} // 匿名命名空间

LedObIndicator::LedObIndicator()
    : indicator_task_(1, TASK_FOREVER, [this]() { Advance(); })
{
    // 回调绑定当前对象；构造期间任务尚未注册，也不会访问 GPIO。
}

LedObIndicator::~LedObIndicator()
{
    // 先从调度器移除任务，避免对象销毁后回调仍访问 this。
    Deinitialize();
}

bool LedObIndicator::IsLitColor(State color)
{
    return color == State::Red || color == State::Green || color == State::Yellow;
}

bool LedObIndicator::SamePattern(const Pattern& next) const
{
    return pattern_.color == next.color && pattern_.on_ms == next.on_ms &&
           pattern_.off_ms == next.off_ms && pattern_.pause_ms == next.pause_ms &&
           pattern_.count == next.count;
}

bool LedObIndicator::StartPattern(const Pattern& next)
{
    if (scheduler_ == nullptr)
    {
        return false;
    }

    // 业务循环可以反复设置同一灯效，正在执行的闪烁不会被重新计时。
    if (indicator_task_.isEnabled() && SamePattern(next))
    {
        return true;
    }

    if (!led::SetState(next.color))
    {
        return false;
    }

    indicator_task_.disable();
    pattern_ = next;
    pulses_started_ = 1;
    light_on_ = true;

    // 首次点亮立即生效，下一次回调负责在亮灯时长结束后熄灭。
    if (!indicator_task_.enableDelayed(pattern_.on_ms))
    {
        led::SetState(State::Off);
        light_on_ = false;
        return false;
    }
    return true;
}

void LedObIndicator::Advance()
{
    if (light_on_)
    {
        if (!led::SetState(State::Off))
        {
            indicator_task_.disable();
            return;
        }
        light_on_ = false;

        // 最后一下之后使用组间停顿，其他脉冲之间使用短间隔。
        const uint32_t wait_ms = pulses_started_ < pattern_.count
                                     ? pattern_.off_ms
                                     : pattern_.pause_ms;
        indicator_task_.delay(wait_ms);
    }
    else
    {
        if (!led::SetState(pattern_.color))
        {
            indicator_task_.disable();
            return;
        }
        light_on_ = true;
        if (pulses_started_ >= pattern_.count)
        {
            pulses_started_ = 0;
        }
        ++pulses_started_;

        // Task::delay() 仅安排下次执行，不阻塞 CPU。
        // 从实际切换时刻计时，主循环偶尔延迟时也不会连续补闪。
        indicator_task_.delay(pattern_.on_ms);
    }
}

bool LedObIndicator::Initialize(Scheduler& runner)
{
    if (scheduler_ != nullptr)
    {
        return scheduler_ == &runner;
    }

    if (!led::Initialize() || !led::SetState(State::Off))
    {
        return false;
    }

    pattern_ = Pattern{};
    pulses_started_ = 0;
    light_on_ = false;
    runner.addTask(indicator_task_);
    scheduler_ = &runner;
    return true;
}

void LedObIndicator::Deinitialize()
{
    if (scheduler_ == nullptr)
    {
        return;
    }

    // 先停止并解除注册，再释放 GPIO；此后重复调用以及析构都不会再次操作调度器。
    indicator_task_.disable();
    scheduler_->deleteTask(indicator_task_);
    scheduler_ = nullptr;
    pattern_ = Pattern{};
    pulses_started_ = 0;
    light_on_ = false;
    led::Deinitialize();
}

bool LedObIndicator::SetSolid(State color)
{
    if (scheduler_ == nullptr || (color != State::Off && !IsLitColor(color)))
    {
        return false;
    }
    if (!led::SetState(color))
    {
        return false;
    }

    indicator_task_.disable();
    pattern_ = Pattern{};
    pattern_.color = color;
    pulses_started_ = 0;
    light_on_ = color != State::Off;
    return true;
}

bool LedObIndicator::SetBlink(State color, uint32_t on_ms, uint32_t off_ms)
{
    if (!IsLitColor(color) || on_ms == 0 || off_ms == 0 ||
        on_ms > max_duration_ms || off_ms > max_duration_ms)
    {
        return false;
    }

    // 普通闪烁是一组只有一下的灯效，组间停顿就是灭灯时长。
    return StartPattern(Pattern{color, on_ms, off_ms, off_ms, 1});
}

bool LedObIndicator::SetBurst(State color, uint8_t count)
{
    if (!IsLitColor(color) || count == 0 || count > max_burst_count)
    {
        return false;
    }

    // 周期 = count 次亮灯 + (count - 1) 次短间隔 + 一次组间停顿。
    const uint32_t pause_ms = burst_period_ms - count * burst_on_ms -
                              (count - 1) * burst_gap_ms;
    return StartPattern(Pattern{color, burst_on_ms, burst_gap_ms, pause_ms, count});
}

} // namespace ind::hardware::led_ob
