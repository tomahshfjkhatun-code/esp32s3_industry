#include <Arduino.h>

// 全工程仅在此处包含调度器实现，其他 .cpp 使用 TaskSchedulerDeclarations.h。
#include <TaskScheduler.h>

#include "src/hardware/led_ob/led_ob_indicator.h"
#include "src/log/log.h"
#include "src/shell/shell.h"

namespace {

constexpr char log_tag[] = "APP";

// 同一文件中先构造调度器，后构造指示灯，确保析构时指示灯先移除自己的任务。
Scheduler scheduler;
::ind::hardware::led_ob::LedObIndicator led_indicator;

} // 匿名命名空间

void setup() {
    if (!::ind::log::Initialize()) {
        return;
    }

    ESP_LOGI(log_tag, "ESP32-S3 start");

    // 其他灯效示例：led_indicator.SetBlink(::ind::hardware::led_ob::State::Green, 3, 990 + 500);
    if (!led_indicator.Initialize(scheduler)) {
        ESP_LOGE(log_tag, "Onboard LED indicator initialization failed");
    } else if (!led_indicator.SetBlink(::ind::hardware::led_ob::State::Green, 20, 1500 - 20)) {
        ESP_LOGE(log_tag, "Onboard LED indicator start failed");
    }

    if (!::ind::shell::Initialize() || !::ind::shell::Start()) {
        ESP_LOGE(log_tag, "Shell initialization failed");
    }
}

void loop() {
    scheduler.execute();
    // 保持调度及时执行，并短暂让出 CPU；Console REPL 仍在独立任务运行。
    delay(1);
}
