#include "shell.h"

#include <Arduino.h>
#include <Console.h>
#include <cstring>

namespace {

constexpr char log_tag[] = "SHELL";

// 全部为常量/零初始化，导出宏的构造函数可在 setup() 前安全收集节点。
IndShellCommand *pending_commands = nullptr;
// 宏导出没有返回值接收者，保留首个错误，等日志初始化后由 Initialize() 报告。
esp_err_t registration_error = ESP_OK;
// 注册与 Initialize() 按接口约定串行调用；原子开关不能单独保护整个链表。
bool registration_closed = false;
bool initialized = false;
bool started = false;

} // namespace

extern "C" esp_err_t ind_shell_register_command(IndShellCommand *command) {
    if (registration_closed) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t error = ESP_OK;
    if (command == nullptr || command->name == nullptr || command->name[0] == '\0' ||
        command->func == nullptr || std::strpbrk(command->name, " \t\r\n\v\f") != nullptr) {
        error = ESP_ERR_INVALID_ARG;
    } else if (std::strcmp(command->name, "help") == 0) {
        error = ESP_ERR_INVALID_STATE;
    } else {
        for (IndShellCommand *entry = pending_commands; entry != nullptr; entry = entry->next) {
            if (entry == command) {
                return ESP_OK;
            }
            if (std::strcmp(entry->name, command->name) == 0) {
                error = ESP_ERR_INVALID_STATE;
                break;
            }
        }

        if (error == ESP_OK) {
            command->next = pending_commands;
            pending_commands = command;
        }
    }

    if (error != ESP_OK && registration_error == ESP_OK) {
        registration_error = error;
    }
    return error;
}

namespace ind::shell {

bool Initialize() {
    if (initialized) {
        return true;
    }

    registration_closed = true;
    if (registration_error != ESP_OK) {
        ESP_LOGE(log_tag, "Command collection failed: %s", esp_err_to_name(registration_error));
        return false;
    }

    if (!Console.begin()) {
        ESP_LOGE(log_tag, "Console initialization failed");
        return false;
    }

    if (!Console.addHelpCmd()) {
        ESP_LOGE(log_tag, "Help command registration failed");
        Console.end();
        return false;
    }

    for (const IndShellCommand *entry = pending_commands; entry != nullptr; entry = entry->next) {
        if (!Console.addCmd(entry->name, entry->help, entry->hint, entry->func)) {
            ESP_LOGE(log_tag, "Command registration failed: %s", entry->name);
            Console.end();
            return false;
        }
    }

    initialized = true;
    return true;
}

bool Start() {
    if (started) {
        return true;
    }

    if (!initialized) {
        ESP_LOGE(log_tag, "Console is not initialized");
        return false;
    }

    ESP_LOGI(log_tag, "Starting Hardware CDC Console");
    if (!Console.attachToSerial(true)) {
        ESP_LOGE(log_tag, "Console REPL task start failed");
        return false;
    }

    started = true;
    return true;
}

} // namespace ind::shell
