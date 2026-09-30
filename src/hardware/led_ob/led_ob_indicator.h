#pragma once

#include "led_ob.h"

// 回调需要捕获 this；此选项在 build_opt.h 中对全工程统一启用，保证 Task 类型一致。
#ifndef _TASK_STD_FUNCTION
#error "LedObIndicator requires -D_TASK_STD_FUNCTION in build_opt.h"
#endif

// 头文件只引入声明，TaskScheduler.h 的实现仍由主 .ino 提供。
#include <TaskSchedulerDeclarations.h>

namespace ind::hardware::led_ob {

// 板载 LED 只由一个已初始化的对象管理，使用期间通过本类设置灯效。
// 本类不提供跨线程同步：所有接口及析构应与 scheduler.execute() 在同一任务执行。
// Shell、通信等其他任务应通过队列将请求交给主循环，不能直接调用这些接口。
class LedObIndicator final {
public:
    // 构造时仅绑定回调，GPIO 和调度任务在 Initialize() 中启用。
    LedObIndicator();
    ~LedObIndicator();

    // 调度器保存 Task 地址，回调捕获 this，因此注册后的对象地址必须保持不变。
    LedObIndicator(const LedObIndicator&) = delete;
    LedObIndicator& operator=(const LedObIndicator&) = delete;
    LedObIndicator(LedObIndicator&&) = delete;
    LedObIndicator& operator=(LedObIndicator&&) = delete;

    // 初始化底层 LED 并注册内部 Task，默认熄灯，不创建独立线程。
    // 同一调度器重复初始化不改变灯效；更换调度器前须先 Deinitialize()。
    // 调度器由调用方管理，必须持续 execute()，并在本对象释放或析构前保持有效。
    bool Initialize(Scheduler& scheduler);

    // 停止灯效、移除内部 Task 并熄灯；析构时也会调用，允许重复释放及重新初始化。
    void Deinitialize();

    // 设置接口在未初始化、参数无效或启动失败时返回 false。
    // 参数无效不改变原灯效；相同闪烁参数重复设置不重置计时。

    // 停止闪烁并常亮指定颜色；State::Off 表示熄灭。
    bool SetSolid(State color);

    // 从亮灯开始，持续循环亮 on_ms、灭 off_ms，时长范围为 1～2147483647 毫秒。
    // 颜色须为 Red/Green/Yellow；快闪 (Red, 100, 100)，心跳 (Green, 50, 950)。
    bool SetBlink(State color, uint32_t on_ms, uint32_t off_ms);

    // 每秒集中闪 count 下后停顿，持续循环；颜色须为 Red/Green/Yellow，count 为 1～3。
    // 每下亮 100ms，组内间隔 100ms，组末分别灭 900/700/500ms。
    // 实际时序受协作式调度的执行延迟影响。
    bool SetBurst(State color, uint8_t count);

private:
    struct Pattern {
        State color = State::Off;
        uint32_t on_ms = 0;
        uint32_t off_ms = 0;
        uint32_t pause_ms = 0;
        uint8_t count = 0;
    };

    static bool IsLitColor(State color);
    bool SamePattern(const Pattern& next) const;
    bool StartPattern(const Pattern& next);
    void Advance();

    Scheduler* scheduler_ = nullptr; // 仅记录注册位置，不拥有调度器。
    Task indicator_task_;
    Pattern pattern_;
    uint8_t pulses_started_ = 0;
    bool light_on_ = false;
};

} // namespace ind::hardware::led_ob
