#include "m31remoteio.h"

#include <ModbusClientTCP.h>
#include <NetworkClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../../log/log.h"

#include <mutex>
#include <new>

namespace ind::devices::m31remoteio {
namespace {

constexpr char log_tag[] = "M31_IO";
constexpr uint16_t ai_count_limit = 1000; // 下一区为 0x03E8 浮点工程量。
constexpr uint16_t ao_count_limit = 500;  // 下一区为 0x01F4 过程量。
constexpr uint32_t max_timeout_ms = 60000;

// 依赖当前 ESP32 3.3.12 / eModbus 1.7.6 的 worker 空闲 delay(1)。
// 不允许更粗的 tick 把该等待变成 vTaskDelay(0)，否则无法确认空闲屏障。
static_assert(configTICK_RATE_HZ == 1000, "M31RemoteIO requires a 1 ms FreeRTOS tick");

} // namespace

namespace detail {

// 只适配 eModbus 的任务生命周期，不重写 TCP 收发或连接协议。
// 本类不注册回调、不并发提交请求、不从外部清队列；升级依赖时须复核：
// worker 先发表同步结果，再在 qLock 内 pop/delete，最后析构局部 response，
// 然后空队列 delay(1)。因此 syncRequest 返回本身不是销毁客户端的屏障。
class ManagedModbusClient final : public ModbusClientTCP {
public:
    explicit ManagedModbusClient(::Client& socket)
        : ModbusClientTCP(socket, 1)
    {
    }

    bool IsStarted() const
    {
        return worker != nullptr;
    }

    void WaitIdle(bool suspend = false)
    {
        if (worker == nullptr)
        {
            return;
        }
        const UBaseType_t previous_priority = uxTaskPriorityGet(worker);
        const UBaseType_t caller_priority = uxTaskPriorityGet(nullptr);
        if (previous_priority < caller_priority)
        {
            vTaskPrioritySet(worker, caller_priority);
        }
        for (;;)
        {
            // 高优先级调用者也须让 worker 有机会运行到空闲；同级 yield
            // 避免两者每 tick 唤醒时，调用者总先检查到尚未运行的 worker。
            taskYIELD();
            bool retired = false;
            bool idle = false;
            {
                std::lock_guard<std::mutex> lock(qLock);
                // 持锁看到空队列说明 pop/delete 与块内日志已完成；此后到
                // delay(1) 只有 millis() 和不阻塞的 vector 析构。eBlocked
                // 因而确认 response 也已释放。没有新投递，醒来仍只会空闲。
                retired = requests.empty();
                if (retired && eTaskGetState(worker) == eBlocked)
                {
                    if (suspend)
                    {
                        vTaskSuspend(worker);
                    }
                    idle = true;
                }
            }
            if (idle)
            {
                if (!suspend && previous_priority < caller_priority)
                {
                    vTaskPrioritySet(worker, previous_priority);
                }
                return;
            }
            if (!retired)
            {
                delay(1); // 必须先解锁，让 worker 完成队列退休。
            }
            // 已退休后只剩极短的析构尾部，直接重新观察，不按 tick 采样；
            // 否则双核上的调用者可能持续错过另一核的 delay(1) 阻塞窗口。
        }
    }

    ModbusMessage Request(const ModbusMessage& request)
    {
        const uint32_t token = ++token_;
        ModbusMessage response = syncRequest(request, token);
        WaitIdle();
        // waitSync 的 60 秒兜底可能早于 worker 收尾。现在已退休，清掉
        // 迟到的同 token 结果，避免下一请求或析构留下未消费响应。
        {
            std::lock_guard<std::mutex> lock(syncRespM);
            syncResponse.erase(token);
        }
        return response;
    }

    void Shutdown()
    {
        WaitIdle(true);
        end(); // 不能持有 qLock 调用；end() 自己也会锁队列。
    }

private:
    uint32_t token_ = 0;
};

} // namespace detail

namespace {

IoResult MapError(Modbus::Error error)
{
    switch (error)
    {
    case Modbus::ILLEGAL_DATA_ADDRESS:
        return IoResult::InvalidChannel;
    case Modbus::INVALID_SERVER:
    case Modbus::ILLEGAL_IP_OR_PORT:
        return IoResult::InvalidConfig;
    case Modbus::IP_CONNECTION_FAILED:
        return IoResult::ConnectFailed;
    case Modbus::TIMEOUT:
        return IoResult::Timeout;
    default:
        return IoResult::ModbusError;
    }
}

uint16_t ReadWord(const ModbusMessage& message, uint16_t offset)
{
    return (static_cast<uint16_t>(message[offset]) << 8) | message[offset + 1];
}

} // namespace

struct M31RemoteIO::Impl {
    // 声明顺序保证 socket 最后析构，始终覆盖 client 保存引用的生命周期。
    NetworkClient socket;
    detail::ManagedModbusClient client;

    Impl(IPAddress ip, uint16_t port, uint32_t connect_timeout_ms,
         uint32_t response_timeout_ms)
        : client(socket)
    {
        socket.setConnectionTimeout(connect_timeout_ms);
        client.setTarget(ip, port, response_timeout_ms);
        // 本机 eModbus 在 timeoutCount > n 时关闭连接；n=1 允许一次超时。
        // 超时计数及关闭时机由库管理，不重放失败请求。
        client.closeConnectionOnTimeouts(1);
        client.begin();
    }

    ~Impl()
    {
        client.Shutdown();
        socket.stop();
    }
};

M31RemoteIO::M31RemoteIO(IPAddress Ip, uint16_t Port, uint8_t UnitId,
                       uint32_t ConnectTimeoutMs, uint32_t ResponseTimeoutMs)
    : ip_(Ip), port_(Port), unit_id_(UnitId),
      connect_timeout_ms_(ConnectTimeoutMs), response_timeout_ms_(ResponseTimeoutMs)
{
}

M31RemoteIO::~M31RemoteIO() = default;

bool M31RemoteIO::IsValidConfig() const
{
    // 当前 M31 使用 IPv4 单播；不接受未指定地址、多播或有限广播。
    // Unit ID 采用实际 eModbus checkServerFC() 支持的 1..247。
    return ip_.type() == IPv4 && ip_[0] != 0 && ip_[0] < 224 &&
           port_ != 0 && unit_id_ >= 1 && unit_id_ <= 247 &&
           connect_timeout_ms_ >= 1 && connect_timeout_ms_ <= max_timeout_ms &&
           response_timeout_ms_ >= 1 && response_timeout_ms_ <= max_timeout_ms;
}

IoResult M31RemoteIO::EnsureInitialized()
{
    if (!IsValidConfig())
    {
        return IoResult::InvalidConfig;
    }
    if (!impl_)
    {
        std::unique_ptr<Impl> candidate;
#if defined(__cpp_exceptions)
        try
        {
            candidate.reset(new (std::nothrow) Impl(ip_, port_, connect_timeout_ms_,
                                                  response_timeout_ms_));
        }
        catch (const std::bad_alloc&)
        {
            return IoResult::InitFailed;
        }
#else
        candidate.reset(new (std::nothrow) Impl(ip_, port_, connect_timeout_ms_,
                                              response_timeout_ms_));
#endif
        if (!candidate || !candidate->client.IsStarted())
        {
            return IoResult::InitFailed;
        }
        impl_ = std::move(candidate);
    }
    return IoResult::Ok;
}

IoResult M31RemoteIO::Exchange(uint8_t function, uint16_t index, uint16_t argument,
                             uint16_t& value)
{
    const IoResult initialized = EnsureInitialized();
    if (initialized != IoResult::Ok)
    {
        return initialized;
    }

    ModbusMessage request;
    if (request.setMessage(unit_id_, function, index, argument) != Modbus::SUCCESS)
    {
        return IoResult::ModbusError;
    }
    ModbusMessage response = impl_->client.Request(request);

    const auto malformed = [this]() {
        // worker 已空闲；丢弃不可信流，下一次调用由 eModbus 重新建连。
        impl_->socket.stop();
        ESP_LOGW(log_tag, "Invalid Modbus response");
        return IoResult::ModbusError;
    };

    if (response.size() < 3 || response[0] != unit_id_)
    {
        return malformed();
    }
    if (response[1] == static_cast<uint8_t>(function | 0x80))
    {
        if (response.size() != 3 || response[2] == 0)
        {
            return malformed();
        }
        const auto error = static_cast<Modbus::Error>(response[2]);
        const IoResult result = MapError(error);
        ESP_LOGW(log_tag, "Modbus FC=0x%02X address=%u error=0x%02X",
                 function, static_cast<unsigned>(index), response[2]);
        // 库仅报告这些帧错误，不关闭 TCP；丢弃不可信流，下一请求按需建连。
        // 普通设备异常、连接失败和超时的连接处理交给 eModbus。
        if (error == Modbus::TCP_HEAD_MISMATCH || error == Modbus::FC_MISMATCH ||
            error == Modbus::SERVER_ID_MISMATCH)
        {
            impl_->socket.stop();
        }
        return result;
    }
    if (response[1] != function)
    {
        return malformed();
    }

    switch (function)
    {
    case Modbus::READ_COIL:
    case Modbus::READ_DISCR_INPUT:
        if (response.size() != 4 || response[2] != 1)
        {
            return malformed();
        }
        value = response[3] & 1U; // 忽略填充位，不能以整个字节非零判断通道。
        return IoResult::Ok;

    case Modbus::READ_HOLD_REGISTER:
    case Modbus::READ_INPUT_REGISTER:
        if (response.size() != 5 || response[2] != 2)
        {
            return malformed();
        }
        value = ReadWord(response, 3);
        return IoResult::Ok;

    case Modbus::WRITE_COIL:
    case Modbus::WRITE_HOLD_REGISTER:
        if (response.size() != 6 || ReadWord(response, 2) != index ||
            ReadWord(response, 4) != argument)
        {
            return malformed();
        }
        return IoResult::Ok;

    default:
        return malformed();
    }
}

IoResult M31RemoteIO::get_di(uint16_t index, bool& value)
{
    uint16_t raw = 0;
    const IoResult result = Exchange(Modbus::READ_DISCR_INPUT, index, 1, raw);
    if (result == IoResult::Ok)
    {
        value = raw != 0;
    }
    return result;
}

IoResult M31RemoteIO::get_do(uint16_t index, bool& value)
{
    uint16_t raw = 0;
    const IoResult result = Exchange(Modbus::READ_COIL, index, 1, raw);
    if (result == IoResult::Ok)
    {
        value = raw != 0;
    }
    return result;
}

IoResult M31RemoteIO::set_do(uint16_t index, bool value)
{
    uint16_t unused = 0;
    return Exchange(Modbus::WRITE_COIL, index, value ? 0xFF00 : 0x0000, unused);
}

IoResult M31RemoteIO::ReadAi(uint16_t index, int32_t& value)
{
    if (index >= ai_count_limit)
    {
        return IoResult::InvalidChannel;
    }
    uint16_t raw = 0;
    const IoResult result = Exchange(Modbus::READ_INPUT_REGISTER, index, 1, raw);
    if (result == IoResult::Ok)
    {
        value = raw < 0x8000 ? static_cast<int32_t>(raw)
                             : static_cast<int32_t>(raw) - 0x10000;
    }
    return result;
}

IoResult M31RemoteIO::get_ai_current(uint16_t index, int32_t& microamps)
{
    return ReadAi(index, microamps);
}

IoResult M31RemoteIO::get_ai_voltage(uint16_t index, int32_t& millivolts)
{
    return ReadAi(index, millivolts);
}

IoResult M31RemoteIO::get_ao_current(uint16_t index, int32_t& microamps)
{
    if (index >= ao_count_limit)
    {
        return IoResult::InvalidChannel;
    }
    uint16_t raw = 0;
    const IoResult result = Exchange(Modbus::READ_HOLD_REGISTER, index, 1, raw);
    if (result == IoResult::Ok)
    {
        microamps = raw;
    }
    return result;
}

IoResult M31RemoteIO::set_ao_current(uint16_t index, int32_t microamps)
{
    if (index >= ao_count_limit)
    {
        return IoResult::InvalidChannel;
    }
    if (microamps < 0 || microamps > 20000)
    {
        return IoResult::InvalidValue;
    }
    uint16_t unused = 0;
    return Exchange(Modbus::WRITE_HOLD_REGISTER, index,
                    static_cast<uint16_t>(microamps), unused);
}

} // namespace ind::devices::m31remoteio
