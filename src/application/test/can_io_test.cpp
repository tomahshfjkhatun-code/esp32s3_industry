#include "can_io_test.h"

#include <ETH.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "../io/702_signal_map.h"
#include "can_io_test_config.h"

namespace ind::application::test {

namespace {
uint32_t NowMs() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }
bool NetworkOk() { return ETH.linkUp() && ETH.hasIP(); }
constexpr uint32_t can_ms = 200;
constexpr uint16_t do_map[4] = {
    io::map::dout::left_bucket_up, io::map::dout::left_bucket_down,
    io::map::dout::right_bucket_up, io::map::dout::right_bucket_down
};
constexpr uint16_t di_map[4] = {
    io::map::di::main_oil_low, io::map::di::main_oil_hot,
    io::map::di::left_filter_alarm, io::map::di::right_filter_alarm
};
constexpr uint16_t ai_map[8] = {
    io::map::ai::left_valve_pos, io::map::ai::left_bucket_pos,
    io::map::ai::left_steer_pos, io::map::ai::left_pressure,
    io::map::ai::right_valve_pos, io::map::ai::right_bucket_pos,
    io::map::ai::right_steer_pos, io::map::ai::right_pressure
};
uint8_t Result(devices::m31remoteio::IoResult result) {
    return result == devices::m31remoteio::IoResult::Ok ? 0 : 0x20U + static_cast<uint8_t>(result);
}
}

Bench& Bench::Instance() {
    static Bench bench;
    return bench;
}

bool Bench::BusOk() const {
    twai_error_state_t state = TWAI_ERROR_BUS_OFF;
    return can::CanService::Instance().State(&state) == ESP_OK && state == TWAI_ERROR_ACTIVE;
}

esp_err_t Bench::Start(const io::NetworkConfig& config) {
    if (task_ != nullptr || queue_ != nullptr) return ESP_ERR_INVALID_STATE;
    network_ = config;
    queue_ = xQueueCreate(1, sizeof(Item));
    if (queue_ == nullptr) return ESP_ERR_NO_MEM;
    if (xTaskCreate(Run, "can_io_bench", 6144, this, 5, &task_) != pdPASS) {
        vQueueDelete(queue_);
        queue_ = nullptr;
        return ESP_ERR_NO_MEM;
    }
    const esp_err_t result = can::CanService::Instance().Initialize(OnRx);
    if (result != ESP_OK) {
        vTaskDelete(task_);
        vQueueDelete(queue_);
        task_ = nullptr;
        queue_ = nullptr;
        return result;
    }
    xTaskNotifyGive(task_);
    return ESP_OK;
}

void Bench::OnRx(const can::Frame* frame) {
    if (frame == nullptr) return;
    auto& self = Instance();
    if (frame->id != rx_id) {
        if (self.active_flag_.load(std::memory_order_relaxed)) self.halt_.store(true, std::memory_order_relaxed);
        return;
    }
    Item item;
    item.at_ms = NowMs();
    item.code = Decode(frame->id, frame->dlc, frame->data, &item.req);
    item.overlap = self.active_flag_.load(std::memory_order_relaxed);
    if (item.code != Code::Ok || item.overlap) self.halt_.store(true, std::memory_order_relaxed);
    if (self.queue_ != nullptr) xQueueOverwrite(self.queue_, &item);
}

void Bench::Send(Op op, uint8_t channel, uint8_t code, uint8_t seq, uint8_t low, uint8_t high) {
    can::Frame frame{};
    frame.id = tx_id;
    frame.dlc = 8;
    Reply(op, channel, code, seq, low, high, frame.data);
    const esp_err_t result = can::CanService::Instance().Send(&frame, 0);
    if (result != ESP_OK) ESP_LOGW("BENCH", "reply queue: %s", esp_err_to_name(result));
}

uint8_t Bench::AllOff(devices::m31remoteio::M31RemoteIO& m31) {
    if (PJ_DO_LEVEL < 0) return static_cast<uint8_t>(Code::Polarity);
    if (!NetworkOk()) return static_cast<uint8_t>(Code::NoNetwork);
    uint8_t first = 0;
    bad_channel_ = 0;
    for (uint8_t channel = 0; channel < 4; ++channel) {
        const auto result = m31.set_do(do_map[channel], PJ_DO_LEVEL == 0);
        if (result != devices::m31remoteio::IoResult::Ok && first == 0) {
            first = Result(result);
            bad_channel_ = channel + 1;
        }
    }
    off_ready_ = first == 0;
    off_fault_ = first != 0;
    if (first != 0) ESP_LOGE("BENCH", "all-off uncertain, M31 code=0x%02x", first);
    return first;
}

void Bench::Stop(devices::m31remoteio::M31RemoteIO& m31, uint8_t phase) {
    if (active_ == 0) return;
    const uint8_t channel = active_;
    const uint8_t seq = pulse_seq_;
    const auto result = m31.set_do(do_map[channel - 1], PJ_DO_LEVEL == 0);
    const uint8_t code = Result(result);
    active_ = 0;
    active_flag_.store(false, std::memory_order_relaxed);
    halt_.store(false, std::memory_order_relaxed);
    if (code != 0) {
        off_ready_ = false;
        off_fault_ = true;
        bad_channel_ = channel;
        ESP_LOGE("BENCH", "DO%u off uncertain, M31 code=0x%02x", channel, code);
    }
    Send(Op::Pulse, channel, code, seq, 0, phase);
}

void Bench::Pulse(devices::m31remoteio::M31RemoteIO& m31, const Request& req) {
    if (PJ_DO_LEVEL < 0) {
        Send(Op::Pulse, req.channel, static_cast<uint8_t>(Code::Polarity), req.seq, 0, 0);
        return;
    }
    if (!NetworkOk()) {
        Send(Op::Pulse, req.channel, static_cast<uint8_t>(Code::NoNetwork), req.seq, 0, 0);
        return;
    }
    if (!BusOk()) {
        Send(Op::Pulse, req.channel, static_cast<uint8_t>(Code::BadCan), req.seq, 0, 0);
        return;
    }
    if (!off_ready_) {
        Send(Op::Pulse, req.channel, static_cast<uint8_t>(Code::Unsafe), req.seq, 0, 0);
        return;
    }
    if (pulse_seen_ && req.seq == last_seq_) {
        Send(Op::Pulse, req.channel, static_cast<uint8_t>(Code::Busy), req.seq, 0, 0);
        return;
    }
    const uint32_t now = NowMs();
    pulse_seen_ = true;
    last_seq_ = req.seq;
    active_ = req.channel;
    pulse_seq_ = req.seq;
    pulse_at_ = now;
    deadline_ = now + req.duration;
    bus_gen_ = can::CanService::Instance().BusGeneration();
    active_flag_.store(true, std::memory_order_relaxed);
    const auto result = m31.set_do(do_map[req.channel - 1], PJ_DO_LEVEL == 1);
    if (result != devices::m31remoteio::IoResult::Ok) {
        Stop(m31, 4);
        Send(Op::Pulse, req.channel, Result(result), req.seq, 0, 4);
        return;
    }
    if (static_cast<int32_t>(NowMs() - deadline_) >= 0 || !BusOk() ||
        can::CanService::Instance().BusGeneration() != bus_gen_ ||
        halt_.load(std::memory_order_relaxed)) {
        Stop(m31, 4);
        Send(Op::Pulse, req.channel, static_cast<uint8_t>(Code::Timing), req.seq, 0, 4);
        return;
    }
    Send(Op::Pulse, req.channel, 0, req.seq, req.channel, 1);
}

void Bench::Handle(devices::m31remoteio::M31RemoteIO& m31, const Item& item) {
    if (active_ != 0) Stop(m31, 3);
    if (item.code != Code::Ok) {
        Send(Op::Ping, 0, static_cast<uint8_t>(Code::BadFrame), 0, 0, 0);
        return;
    }
    const auto& req = item.req;
    if (req.op == Op::Pulse && item.overlap) {
        Send(req.op, req.channel, static_cast<uint8_t>(Code::Busy), req.seq, 0, 0);
        return;
    }
    if (req.op == Op::Pulse && static_cast<uint32_t>(NowMs() - item.at_ms) > 200) {
        Send(req.op, req.channel, static_cast<uint8_t>(Code::Stale), req.seq, 0, 0);
        return;
    }
    switch (req.op) {
    case Op::Ping: {
        const uint8_t flags = (NetworkOk() ? 1U : 0U) | (PJ_DO_LEVEL >= 0 ? 2U : 0U) |
            (off_ready_ ? 4U : 0U) | (off_fault_ ? 8U : 0U) | (BusOk() ? 16U : 0U) | 32U;
        Send(req.op, 0, 0, req.seq, flags, off_fault_ ? bad_channel_ : active_);
        break;
    }
    case Op::Di: {
        if (!NetworkOk()) {
            Send(req.op, 0, static_cast<uint8_t>(Code::NoNetwork), req.seq, 0, 0);
            break;
        }
        uint8_t states = 0;
        uint8_t valid = 0;
        uint8_t first = 0;
        for (uint8_t i = 0; i < 4; ++i) {
            bool value = false;
            const auto result = m31.get_di(di_map[i], value);
            if (result == devices::m31remoteio::IoResult::Ok) {
                valid |= 1U << i;
                if (value) states |= 1U << i;
            } else if (first == 0) first = Result(result);
        }
        Send(req.op, 0, first, req.seq, states, valid);
        break;
    }
    case Op::Ai: {
        if (!NetworkOk()) {
            Send(req.op, req.channel, static_cast<uint8_t>(Code::NoNetwork), req.seq, 0xFF, 0xFF);
            break;
        }
        const uint8_t first = req.channel == 0 ? 1 : req.channel;
        const uint8_t last = req.channel == 0 ? 8 : req.channel;
        for (uint8_t channel = first; channel <= last; ++channel) {
            int32_t current = 0;
            const auto result = m31.get_ai_current(ai_map[channel - 1], current);
            const uint16_t raw = result == devices::m31remoteio::IoResult::Ok ? static_cast<uint16_t>(current) : 0xFFFFU;
            Send(req.op, channel, Result(result), req.seq, static_cast<uint8_t>(raw), static_cast<uint8_t>(raw >> 8));
        }
        break;
    }
    case Op::Pulse:
        Pulse(m31, req);
        break;
    case Op::Off: {
        const uint8_t code = AllOff(m31);
        Send(req.op, 0, code, req.seq, bad_channel_, 0);
        break;
    }
    }
}

void Bench::Run(void* context) {
    auto* self = static_cast<Bench*>(context);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const auto& config = self->network_;
    devices::m31remoteio::M31RemoteIO m31(config.m31_address, config.m31_port, config.m31_unit, config.connect_ms, config.response_ms);
    for (;;) {
        const uint32_t now = NowMs();
        if (self->active_ != 0 &&
            (static_cast<int32_t>(now - self->deadline_) >= 0 ||
             now - self->pulse_at_ >= can_ms ||
             self->halt_.load(std::memory_order_relaxed) ||
             !self->BusOk() ||
             can::CanService::Instance().BusGeneration() != self->bus_gen_ ||
             !NetworkOk())) self->Stop(m31, 2);
        if (!NetworkOk()) self->off_ready_ = false;
        if (self->active_ == 0 && !self->off_ready_ && PJ_DO_LEVEL >= 0 &&
            NetworkOk() && now - self->retry_at_ >= 250) {
            self->retry_at_ = now;
            self->AllOff(m31);
        }
        Item item;
        if (xQueueReceive(self->queue_, &item, pdMS_TO_TICKS(self->active_ != 0 ? 1 : 10)) == pdTRUE) self->Handle(m31, item);
    }
}

} // namespace ind::application::test
