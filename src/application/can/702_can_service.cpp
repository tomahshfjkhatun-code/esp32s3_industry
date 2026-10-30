#include "702_can_service.h"
#include "../test/can_io_test_config.h"

#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"

#define CAN_TX_DEPTH 16
#define CAN_STACK_BYTES 4096
#define CAN_TX_PRIO 6
#define CAN_RX_PRIO 7
#define CAN_BITRATE 250000

namespace ind::application::can {

CanService& CanService::Instance() {
    static CanService service;
    return service;
}

void CanService::OnError(void* pump_context, twai_error_state_t old_state, twai_error_state_t new_state, const Esp32TwaiStatistics* pump_statistics) {
    (void)old_state;
    (void)pump_statistics;
    if (new_state != TWAI_ERROR_ACTIVE) static_cast<CanService*>(pump_context)->bus_generation_.fetch_add(1, std::memory_order_relaxed);
}

uint32_t CanService::BusGeneration() const { return bus_generation_.load(std::memory_order_relaxed); }

bool CanService::RxValid(const twai_frame_t* frame) {
    if (frame->header.ide != 1 ||
        frame->header.rtr != 0 ||
        frame->header.fdf != 0 ||
        frame->header.brs != 0 ||
        frame->header.esi != 0) {
        return false;
    }

    #if PJ_BENCH_MODE
    // 台架模式将全部扩展数据帧交给独立协议层校验；其它 ID 只触发安全停止。
    return frame->header.dlc <= 8;
    #endif

    // 本控制板接收无人系统的左右机指令。
    // 协议字段覆盖字节 0 到 7，此示例按 DLC=8 检查。
    switch (frame->header.id) {
    case 0x04069001U:
    case 0x04069081U:
    case 0x04069002U:
    case 0x04069082U:
        return frame->header.dlc == 8;
    default:
        return false;
    }
}

void CanService::TxTask(void* arg) {
    auto* self = static_cast<CanService*>(arg);
    TxItem pump_item;

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    for (;;) {
        if (xQueueReceive(self->txq_, &pump_item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        twai_error_state_t pump_state = TWAI_ERROR_BUS_OFF;
        if (now_ms - pump_item.queued_at > 200 || self->State(&pump_state) != ESP_OK || pump_state != TWAI_ERROR_ACTIVE) continue;
        Frame& msg = pump_item.pump_frame;
        twai_frame_t frame = {};
        frame.header.id = msg.id;
        frame.header.dlc = msg.dlc;
        frame.header.ide = 1;
        frame.header.rtr = 0;
        frame.buffer = msg.data;
        frame.buffer_len = msg.dlc;

        // msg.data 在本次 send 返回前一直由 TX 任务持有。
        esp_err_t err = esp32_twai_send(&frame, 500);
        if (err != ESP_OK) {
            ESP_LOGW("CAN", "TX id=0x%08lx result=%s",
                     (unsigned long)msg.id, esp_err_to_name(err));
        }
    }
}

void CanService::RxTask(void* arg) {
    auto* self = static_cast<CanService*>(arg);
    uint8_t data[8];
    twai_frame_t frame = {};

    frame.buffer = data;
    frame.buffer_len = sizeof(data);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    for (;;) {
        esp_err_t err = esp32_twai_receive(&frame, 100);
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (!RxValid(&frame)) {
            continue;
        }

        Frame msg = {};
        msg.at_ms = static_cast<uint32_t>(frame.header.timestamp / 1000U);
        msg.id = frame.header.id;
        msg.dlc = (uint8_t)frame.header.dlc;
        std::memcpy(msg.data, data, msg.dlc);
        self->on_rx_(&msg);
    }
}

void CanService::Cleanup() {
    // 仅在 open 尚未成功时调用。
    if (rx_task_ != nullptr) vTaskDelete(rx_task_);
    if (tx_task_ != nullptr) vTaskDelete(tx_task_);
    if (txq_ != nullptr) vQueueDelete(txq_);

    rx_task_ = nullptr;
    tx_task_ = nullptr;
    txq_ = nullptr;
    on_rx_ = nullptr;
}

esp_err_t CanService::Initialize(RxHandler on_rx) {
    if (on_rx == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (ready_ || txq_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    on_rx_ = on_rx;
    txq_ = xQueueCreate(CAN_TX_DEPTH, sizeof(TxItem));
    if (txq_ == nullptr) {
        Cleanup();
        return ESP_ERR_NO_MEM;
    }

    // 两个任务先等待通知；open 成功前都不会调用底层收发。
    // TX 固定到当前核，确保与下方 open 的调用核相同。
    BaseType_t core = xPortGetCoreID();
    if (xTaskCreatePinnedToCore(TxTask, "can_tx", CAN_STACK_BYTES, this, CAN_TX_PRIO, &tx_task_, core) != pdPASS) {
        Cleanup();
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(RxTask, "can_rx", CAN_STACK_BYTES, this, CAN_RX_PRIO, &rx_task_) != pdPASS) {
        Cleanup();
        return ESP_ERR_NO_MEM;
    }

    twai_onchip_node_config_t cfg = {};
    cfg.io_cfg.tx = GPIO_NUM_15;
    cfg.io_cfg.rx = GPIO_NUM_16;
    cfg.io_cfg.quanta_clk_out = (gpio_num_t)-1;
    cfg.io_cfg.bus_off_indicator = (gpio_num_t)-1;
    cfg.bit_timing.bitrate = CAN_BITRATE;
    cfg.tx_queue_depth = 1;
    cfg.fail_retry_cnt = -1;

    esp_err_t err = esp32_twai_open(&cfg, OnError, this);
    if (err != ESP_OK) {
        Cleanup();
        return err;
    }

    ready_ = true;
    xTaskNotifyGive(tx_task_);
    xTaskNotifyGive(rx_task_);
    return ESP_OK;
}

esp_err_t CanService::Send(const Frame* msg, uint32_t wait_ms) {
    if (msg == nullptr || msg->dlc > 8 || msg->id > TWAI_EXT_ID_MASK || wait_ms > 60000) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ready_) {
        return ESP_ERR_INVALID_STATE;
    }

    Frame copy = {};
    copy.id = msg->id;
    copy.dlc = msg->dlc;
    std::memcpy(copy.data, msg->data, msg->dlc);

    const TxItem pump_item{copy, static_cast<uint32_t>(esp_timer_get_time() / 1000)};
    if (xQueueSend(txq_, &pump_item, pdMS_TO_TICKS(wait_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // 只表示应用 TX 队列已接收，并非总线已确认发送。
    return ESP_OK;
}

esp_err_t CanService::State(twai_error_state_t* state) const {
    if (state == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ready_) {
        return ESP_ERR_INVALID_STATE;
    }

    twai_node_status_t status;
    esp_err_t err = esp32_twai_get_info(&status, nullptr, nullptr);
    if (err == ESP_OK) {
        *state = status.state;
    }
    return err;
}

esp_err_t CanService::Statistics(Esp32TwaiStatistics* stats) const {
    if (stats == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ready_) {
        return ESP_ERR_INVALID_STATE;
    }

    return esp32_twai_get_info(nullptr, nullptr, stats);
}

} // namespace ind::application::can
