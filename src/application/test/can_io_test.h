#pragma once

#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "../can/702_can_service.h"
#include "../io/702_io_service.h"
#include "can_io_test_proto.h"

namespace ind::application::test {

class Bench final {
public:
    static Bench& Instance();
    esp_err_t Start(const io::NetworkConfig& config);

private:
    Bench() = default;

    struct Item {
        Request req;
        Code code = Code::Ok;
        uint32_t at_ms = 0;
        bool overlap = false;
    };

    static void OnRx(const can::Frame* frame);
    static void Run(void* context);
    void Handle(devices::m31remoteio::M31RemoteIO& m31, const Item& item);
    void Pulse(devices::m31remoteio::M31RemoteIO& m31, const Request& req);
    void Stop(devices::m31remoteio::M31RemoteIO& m31, uint8_t phase);
    uint8_t AllOff(devices::m31remoteio::M31RemoteIO& m31);
    void Send(Op op, uint8_t channel, uint8_t code, uint8_t seq, uint8_t low, uint8_t high);
    bool BusOk() const;

    QueueHandle_t queue_ = nullptr;
    TaskHandle_t task_ = nullptr;
    io::NetworkConfig network_;
    std::atomic<bool> halt_{false};
    std::atomic<bool> active_flag_{false};
    bool off_ready_ = false;
    bool off_fault_ = false;
    uint8_t bad_channel_ = 0;
    uint8_t active_ = 0;
    uint8_t pulse_seq_ = 0;
    uint8_t last_seq_ = 0;
    bool pulse_seen_ = false;
    uint32_t deadline_ = 0;
    uint32_t pulse_at_ = 0;
    uint32_t retry_at_ = 0;
    uint32_t bus_gen_ = 0;
};

} // namespace ind::application::test
