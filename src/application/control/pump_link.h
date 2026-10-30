#pragma once

#include <atomic>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "../can/702_can_service.h"
#include "pump_control.h"

namespace ind::application::control {

using EncodeResult = can::EncodeResult;

struct Counters {
    // 仅统计 CAN 入队失败的新帧，不包含溢出后主动清空的旧事件。
    uint32_t rx_dropped;
    uint32_t reset_done;
    uint32_t reset_rejected;
    uint32_t not_ready;
    uint32_t encode_failed;
    uint32_t enqueue_failed;
    uint32_t queued;
};

// 状态数据由 IO/发动机等采集层提供；字段缺失用 NotReady，数据/编码错误用 Invalid。
// slot 0/1/2 对应状态；slot 3/4 使用同一报警 CAN ID 发送不同 SPN/FMI。
class StatusCodec {
public:
    virtual ~StatusCodec() = default;
    virtual EncodeResult Encode(const Snapshot& status, uint8_t slot, can::Frame* frame) = 0;
};

// 唯一的 CAN 应用连接器。Start 应在 CAN 初始化核调用一次；对象须永久存活。
// RX 回调只复制帧入队，控制任务消费；TWAI TX/RX 专用任务由 CanService 保持。
class PumpLink final : private CanPort {
public:
    PumpLink(IoPort& io, StatusCodec& codec);

    PumpLink(const PumpLink&) = delete;
    PumpLink& operator=(const PumpLink&) = delete;

    esp_err_t Start(const Settings& cfg);
    esp_err_t Start(const Settings& left_config, const Settings& right_config);
    // ESP_OK 只表示请求入队；复位是否满足条件由控制任务判断，通过计数查询结果。
    esp_err_t RequestReset(Side side);
    Counters Statistics() const;
    Snapshot Status(Side pump_side) const;
    bool BusReady() const;

private:
    enum class ItemKind : uint8_t { Can, Reset };
    struct RxItem {
        ItemKind kind;
        can::Frame frame;
        uint32_t at_ms;
        Side side;
    };

    static void OnRx(const can::Frame* frame);
    static void ControlTask(void* arg);
    static uint32_t NowMs();
    bool Publish(const Snapshot& status) override;
    bool HandleOverflow();

    mutable portMUX_TYPE pump_lock_ = portMUX_INITIALIZER_UNLOCKED;
    Snapshot left_status_;
    Snapshot right_status_;
    std::atomic<bool> pump_bus_{false};
    StatusCodec& codec_;
    PumpRuntime runtime_;
    QueueHandle_t rxq_ = nullptr;
    TaskHandle_t task_ = nullptr;
    std::atomic<bool> started_{false};
    std::atomic<bool> overflow_{false};
    std::atomic<uint32_t> rx_dropped_{0};
    std::atomic<uint32_t> reset_done_{0};
    std::atomic<uint32_t> reset_rejected_{0};
    std::atomic<uint32_t> not_ready_{0};
    std::atomic<uint32_t> encode_failed_{0};
    std::atomic<uint32_t> enqueue_failed_{0};
    std::atomic<uint32_t> queued_{0};

    static std::atomic<PumpLink*> active_;
};

} // namespace ind::application::control
