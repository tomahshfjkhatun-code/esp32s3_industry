#include "shell.h"

#include <Arduino.h>
#include <Console.h>

namespace ind::shell {
namespace {

constexpr char log_tag[] = "SHELL";
constexpr char prompt[] = "industry> ";

bool initialized = false;
bool started = false;

} // namespace

bool Initialize()
{
    if (initialized)
    {
        return true;
    }

    Console.setPrompt(prompt);
    Console.setMaxHistory(32);

    // PuTTY 等终端支持 VT100。固定使用完整模式可以避免设备先于终端连接
    // 时，自动探测超时后永久退回不带补全和历史浏览的 plain mode。
    Console.setPlainMode(false, true);

    if (!Console.begin())
    {
        ESP_LOGE(log_tag, "Console initialization failed");
        return false;
    }

    if (!Console.addHelpCmd())
    {
        ESP_LOGE(log_tag, "Help command registration failed");
        Console.end();
        return false;
    }

    initialized = true;
    return true;
}

bool Start()
{
    if (started)
    {
        return true;
    }

    if (!initialized)
    {
        ESP_LOGE(log_tag, "Console is not initialized");
        return false;
    }

    ESP_LOGI(log_tag, "Starting Hardware CDC Console");
    if (!Console.attachToSerial(true))
    {
        ESP_LOGE(log_tag, "Console REPL task start failed");
        return false;
    }

    started = true;
    return true;
}

} // namespace ind::shell
