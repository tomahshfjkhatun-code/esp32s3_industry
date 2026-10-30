#include "pump_link.h"

#include "esp_timer.h"

namespace ind::application::control {

namespace {

constexpr uint8_t rx_depth = 16;
constexpr uint32_t task_stack = 4096;
constexpr UBaseType_t task_prio = 5;
constexpr uint32_t cycle_ms = 20;
static_assert(pdMS_TO_TICKS(cycle_ms) > 0, "control period must be at least one RTOS tick");

uint32_t StatusId(Side side, uint8_t slot) {
    const uint32_t offset = side == Side::Left ? 0U : 0x80U;
    switch (slot) {
    case 0: return 0x06069001U + offset;
    case 1: return 0x06069002U + offset;
    case 2: return 0x06069003U + offset;
    case 3:
    case 4: return 0x0A069001U + offset;
    default: return 0;
    }
}

} // namespace

std::atomic<PumpLink*> PumpLink::active_{nullptr};

PumpLink::PumpLink(IoPort& io, StatusCodec& codec) : codec_(codec), runtime_(io, *this) {
    left_status_.side = Side::Left;
    right_status_.side = Side::Right;
}

uint32_t PumpLink::NowMs() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

void PumpLink::OnRx(const can::Frame* frame) {
    PumpLink* self = active_.load(std::memory_order_acquire);
    if (self == nullptr || frame == nullptr || self->rxq_ == nullptr) {
        return;
    }
    if (!PumpControl::ValidCan(frame->id, frame->dlc, frame->data)) return;

    RxItem item = {};
    item.kind = ItemKind::Can;
    item.frame = *frame;
    item.at_ms = frame->at_ms;
    // 回调运行在 CanService 的常驻 RX 任务中，不能等待控制任务或阻塞 bus-off 恢复。
    if (xQueueSend(self->rxq_, &item, 0) != pdTRUE) {
        self->rx_dropped_.fetch_add(1, std::memory_order_relaxed);
        self->overflow_.store(true, std::memory_order_release);
        xTaskNotifyGive(self->task_);
    }
}

bool PumpLink::HandleOverflow() {
    if (!overflow_.exchange(false, std::memory_order_acq_rel)) return false;
    // 清掉旧授权、旧运动和旧复位事件；重置队列时新到的帧也可能丢弃，按失控保护处理。
    xQueueReset(rxq_);
    runtime_.ReportFault(FaultCode::RxOverflow, NowMs());
    return true;
}

void PumpLink::ControlTask(void* arg) {
    auto* self = static_cast<PumpLink*>(arg);
    while (!self->started_.load(std::memory_order_acquire)) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    uint32_t bus_generation = 0;
    uint64_t bus_dropped = 0;
    for (;;) {
        twai_error_state_t pump_state = TWAI_ERROR_BUS_OFF;
        const bool pump_ready = can::CanService::Instance().State(&pump_state) == ESP_OK && pump_state == TWAI_ERROR_ACTIVE;
        const uint32_t bus_current = can::CanService::Instance().BusGeneration();
        const bool bus_changed = bus_current != bus_generation;
        bus_generation = bus_current;
        if (bus_changed) self->runtime_.ReportFault(FaultCode::CanBus, NowMs());
        Esp32TwaiStatistics bus_statistics{};
        if (can::CanService::Instance().Statistics(&bus_statistics) == ESP_OK && bus_statistics.rx_dropped != bus_dropped) {
            bus_dropped = bus_statistics.rx_dropped;
            self->overflow_.store(true, std::memory_order_release);
        }
        self->runtime_.SetBus(pump_ready, NowMs());
        self->pump_bus_.store(pump_ready, std::memory_order_release);
        if (!pump_ready || bus_changed) xQueueReset(self->rxq_);
        RxItem item;
        const bool overflow = self->HandleOverflow();
        for (uint8_t i = 0; !overflow && i < rx_depth && xQueueReceive(self->rxq_, &item, 0) == pdTRUE; ++i) {
            if (self->HandleOverflow()) break;
            if (item.kind == ItemKind::Can) {
                self->runtime_.ReceiveCan(item.frame.id, item.frame.dlc, item.frame.data, item.at_ms);
            } else if (self->runtime_.ResetFault(item.side, NowMs())) {
                self->reset_done_.fetch_add(1, std::memory_order_relaxed);
            } else {
                self->reset_rejected_.fetch_add(1, std::memory_order_relaxed);
            }
        }
        self->HandleOverflow();
        self->runtime_.Cycle(NowMs());
        portENTER_CRITICAL(&self->pump_lock_);
        self->left_status_ = self->runtime_.Status(Side::Left);
        self->right_status_ = self->runtime_.Status(Side::Right);
        portEXIT_CRITICAL(&self->pump_lock_);
        // 溢出或复位通知会提前唤醒；真实最大停机延迟还取决于 IO 方法和任务调度耗时。
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(cycle_ms));
    }
}

esp_err_t PumpLink::Start(const Settings& cfg) {
    return Start(cfg, cfg);
}

esp_err_t PumpLink::Start(const Settings& left_config, const Settings& right_config) {
    if (started_.load(std::memory_order_acquire) || active_.load(std::memory_order_acquire) != nullptr || rxq_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!runtime_.Configure(left_config, right_config)) {
        return ESP_ERR_INVALID_ARG;
    }

    rxq_ = xQueueCreate(rx_depth, sizeof(RxItem));
    if (rxq_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(ControlTask, "pump_ctl", task_stack, this, task_prio, &task_) != pdPASS) {
        vQueueDelete(rxq_);
        rxq_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    // CanService 在当前核调用 esp32_twai_open，并把 TX 任务固定在同一核；
    // 其 RX 任务独立常驻，本连接器不直接碰 esp32_twai_send/receive。
    active_.store(this, std::memory_order_release);
    const esp_err_t err = can::CanService::Instance().Initialize(OnRx);
    if (err != ESP_OK) {
        // 当前 CanService 只有 open 成功才通知 RX 执行，因此错误返回时尚无用户回调。
        // 若将来改为运行中关闭/注销回调，必须先等待回调退出才能释放队列。
        active_.store(nullptr, std::memory_order_release);
        vTaskDelete(task_);
        vQueueDelete(rxq_);
        task_ = nullptr;
        rxq_ = nullptr;
        return err;
    }

    started_.store(true, std::memory_order_release);
    xTaskNotifyGive(task_);
    return ESP_OK;
}

bool PumpLink::Publish(const Snapshot& status) {
    if (!pump_bus_.load(std::memory_order_acquire)) return false;
    bool queued = false;
    bool failed = false;
    constexpr uint8_t slots[5] = {3, 4, 0, 1, 2};
    for (uint8_t index = 0; index < 5; ++index) {
        const uint8_t slot = slots[index];
        can::Frame frame = {};
        const EncodeResult result = codec_.Encode(status, slot, &frame);
        if (result == EncodeResult::NotReady) {
            not_ready_.fetch_add(1, std::memory_order_relaxed);
            continue;
        }
        if (result != EncodeResult::Ready || frame.id != StatusId(status.side, slot) || frame.dlc != 8) {
            encode_failed_.fetch_add(1, std::memory_order_relaxed);
            failed = true;
            continue;
        }
        // Send 只保证进入 CanService 的 TX 队列，不代表总线已确认发送。
        if (can::CanService::Instance().Send(&frame, 0) != ESP_OK) {
            enqueue_failed_.fetch_add(1, std::memory_order_relaxed);
            failed = true;
            continue;
        }
        queued_.fetch_add(1, std::memory_order_relaxed);
        queued = true;
    }
    return queued && !failed;
}

esp_err_t PumpLink::RequestReset(Side side) {
    if (side != Side::Left && side != Side::Right) return ESP_ERR_INVALID_ARG;
    if (!started_.load(std::memory_order_acquire) || !BusReady()) return ESP_ERR_INVALID_STATE;
    RxItem item = {};
    item.kind = ItemKind::Reset;
    item.side = side;
    if (xQueueSend(rxq_, &item, 0) != pdTRUE) return ESP_ERR_TIMEOUT;
    xTaskNotifyGive(task_);
    return ESP_OK;
}

Snapshot PumpLink::Status(Side pump_side) const {
    portENTER_CRITICAL(&pump_lock_);
    const Snapshot pump_status = pump_side == Side::Left ? left_status_ : right_status_;
    portEXIT_CRITICAL(&pump_lock_);
    return pump_status;
}

bool PumpLink::BusReady() const {
    return pump_bus_.load(std::memory_order_acquire);
}

Counters PumpLink::Statistics() const {
    // 每项计数独立读取，诊断快照不承诺所有计数处于同一时刻。
    return {rx_dropped_.load(std::memory_order_relaxed), reset_done_.load(std::memory_order_relaxed),
            reset_rejected_.load(std::memory_order_relaxed), not_ready_.load(std::memory_order_relaxed),
            encode_failed_.load(std::memory_order_relaxed), enqueue_failed_.load(std::memory_order_relaxed),
            queued_.load(std::memory_order_relaxed)};
}

} // namespace ind::application::control
