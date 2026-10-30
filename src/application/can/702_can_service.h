#pragma once

#include <stdint.h>
#include <atomic>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "../../hardware/twai/esp32_twai_internal.h"

namespace ind::application::can {

struct Frame {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint32_t at_ms = 0;
};

// 回调在 RX 任务中执行，必须快速返回；需要留存报文时由接收方复制。
using RxHandler = void (*)(const Frame* msg);

// 底层 TWAI 只有一个控制器且没有 close，服务对象也只允许一个实例。
class CanService final {
public:
    static CanService& Instance();

    CanService(const CanService&) = delete;
    CanService& operator=(const CanService&) = delete;

    esp_err_t Initialize(RxHandler on_rx);
    esp_err_t Send(const Frame* msg, uint32_t wait_ms);
    esp_err_t State(twai_error_state_t* state) const;
    esp_err_t Statistics(Esp32TwaiStatistics* stats) const;
    uint32_t BusGeneration() const;

private:
    CanService() = default;
    ~CanService() = default;

    static void OnError(void* pump_context, twai_error_state_t old_state, twai_error_state_t new_state, const Esp32TwaiStatistics* pump_statistics);
    static bool RxValid(const twai_frame_t* frame);
    static void TxTask(void* arg);
    static void RxTask(void* arg);
    void Cleanup();

    struct TxItem { Frame pump_frame; uint32_t queued_at; };
    QueueHandle_t txq_ = nullptr;
    TaskHandle_t tx_task_ = nullptr;
    TaskHandle_t rx_task_ = nullptr;
    RxHandler on_rx_ = nullptr;
    bool ready_ = false;
    std::atomic<uint32_t> bus_generation_{0};
};

} // namespace ind::application::can
