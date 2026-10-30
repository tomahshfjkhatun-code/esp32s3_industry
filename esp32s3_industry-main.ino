#include <Console.h>
#include <Arduino.h>
#include <WiFi.h>
#include <esp32-hal-bt.h>

// 全工程仅在此处包含调度器实现，其他 .cpp 使用 TaskSchedulerDeclarations.h。
#include <TaskScheduler.h>

#include "src/hardware/led_ob/led_ob_indicator.h"
#include "src/hardware/w5500/w5500.h"
#include "src/log/log.h"
#include "src/application/702_app.h"
#include "src/application/test/can_io_test_config.h"
#if PJ_BENCH_MODE
#include "src/application/test/can_io_test.h"
#endif
#include "src/shell/shell.h"

namespace {

constexpr char log_tag[] = "APP";

// 同一文件中先构造调度器，后构造指示灯，确保析构时指示灯先移除自己的任务。
Scheduler scheduler;
::ind::hardware::led_ob::LedObIndicator led_indicator;

} // 匿名命名空间

void setup() {
    if (!btStop()) {
        ESP_LOGE(log_tag, "Failed to disable Bluetooth");
    }

    if (!WiFi.mode(WIFI_OFF)) {
        ESP_LOGE(log_tag, "Failed to disable Wi-Fi");
    }

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

    // 使用模块默认静态 IPv4 配置，不等待网线连接或地址就绪。
    if (!::ind::hardware::w5500::Initialize()) {
        ESP_LOGE(log_tag, "W5500 initialization failed");
    }

#if PJ_BENCH_MODE
    const esp_err_t bench_result = ::ind::application::test::Bench::Instance().Start(::ind::application::Make702Config().m31_network);
    if (bench_result != ESP_OK) ESP_LOGE(log_tag, "CAN/M31 bench startup: %s", esp_err_to_name(bench_result));
#else
    const esp_err_t pump_result = ::ind::application::App702::Instance().Start(::ind::application::Make702Config());
    if (pump_result != ESP_OK) ESP_LOGE(log_tag, "702 startup: %s", esp_err_to_name(pump_result));
#endif

    if (!::ind::shell::Initialize() || !::ind::shell::Start()) {
        ESP_LOGE(log_tag, "Shell initialization failed");
    }
}

void loop() {
    scheduler.execute();
    // 保持调度及时执行，并短暂让出 CPU；Console REPL 仍在独立任务运行。
    delay(1);
}
