#include "log.h"

#include <Arduino.h>
#include <esp_log.h>

#include <atomic>
#include <stdarg.h>
#include <stdio.h>

#if !ARDUINO_USB_MODE || !ARDUINO_USB_CDC_ON_BOOT
#error "Select USB Mode: Hardware CDC and USB CDC On Boot: Enabled"
#endif

namespace ind::log {
namespace {

// ESP-IDF 默认使用 vprintf。预先保存该值，可以覆盖安装新回调后的极短并发窗口。
// 当前固定的 ESP32-S3 工具链会将这个自然对齐的原子读写直接内联。
alignas(sizeof(vprintf_like_t)) std::atomic<vprintf_like_t> previous_vprintf{
    &::vprintf};
bool initialized = false;

static_assert(
    ATOMIC_POINTER_LOCK_FREE != 0,
    "the target must support lock-free pointer atomics");

int RouteVprintf(const char *format, va_list arguments)
{
    if (format == nullptr)
    {
        return -1;
    }

    const vprintf_like_t console_vprintf =
        previous_vprintf.load(std::memory_order_acquire);

    if (console_vprintf == nullptr || console_vprintf == RouteVprintf)
    {
        return 0;
    }

    // 每个输出端都必须使用独立的 va_list 副本。
    // 将来增加文件和网络日志时，也必须分别复制后再使用。
    va_list console_arguments;
    va_copy(console_arguments, arguments);
    const int result = console_vprintf(format, console_arguments);
    va_end(console_arguments);
    return result;
}

} // 匿名命名空间

bool Initialize()
{
    if (initialized)
    {
        return true;
    }

    // 该路径用于直接使用 Serial，以及 ets_printf、ISR 等旁路诊断输出。
    // 普通 ESP_LOG 日志仍然通过下面捕获的旧 IDF Console 回调输出。
    Serial.begin();
    Serial.setDebugOutput(true);

    // 运行时日志等级与 Arduino IDE 的 Core Debug Level 保持一致。
    esp_log_level_set("*", static_cast<esp_log_level_t>(ARDUHAL_LOG_LEVEL));

    // 全工程只允许安装一次。旧回调会永久作为 Console 后端；
    // 将来的文件和网络日志必须在 RouteVprintf() 内部分流，不能再次注册。
    const vprintf_like_t captured_vprintf = esp_log_set_vprintf(RouteVprintf);
    if (captured_vprintf != nullptr && captured_vprintf != RouteVprintf)
    {
        previous_vprintf.store(captured_vprintf, std::memory_order_release);
    }
    initialized = true;

    return true;
}

} // 命名空间 ind::log
