#pragma once

#include <esp32-hal-log.h>

#include <stdint.h>

#ifndef USE_ESP_IDF_LOG
#error "Logging requires -DUSE_ESP_IDF_LOG in the sketch build_opt.h"
#endif

namespace ind::log {

// 初始化 Arduino Hardware CDC，并安装全工程唯一的 ESP-IDF 日志回调。
// 初始化时捕获的旧回调会在程序整个生命周期内继续作为 Console 后端。
// 仅从 setup() 初始化路径调用；可以重复调用，但只会执行一次实际初始化。
bool Initialize();

} // 命名空间 ind::log
