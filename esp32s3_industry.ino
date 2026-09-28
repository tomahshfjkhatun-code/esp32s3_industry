#include <Arduino.h>

#include "src/log/log.h"
#include "src/shell/shell.h"

namespace {

constexpr char log_tag[] = "APP";

} // 匿名命名空间

void setup() {
    if (!::ind::log::Initialize()) {
        return;
    }

    ESP_LOGI(log_tag, "ESP32-S3 start");

    if (!::ind::shell::Initialize() || !::ind::shell::Start()) {
        ESP_LOGE(log_tag, "Shell initialization failed");
    }
}

void loop() {
    // Console REPL runs in its own FreeRTOS task.
    delay(1000);
}
