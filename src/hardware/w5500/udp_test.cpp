#include "../../log/log.h"
#include "../../shell/shell.h"

#include <ETH.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <lwip/sockets.h>

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// W5500 初始化成功后注册 Shell 命令，不自动启动测试。
// 命令用法：udp_test start [port] / udp_test stop，默认端口 5000。
// W5500 由主程序初始化，命令仅控制 UDP 测试的工作任务和 Socket。
// 默认关闭；首个有效数据报登记心跳对端，此后每秒发送 HEARTBEAT。
// 回环格式：ECHO seq=<序号> rx_ms=<开机毫秒> len=<负载字节数>\n<原始数据>。
// 支持空数据报和二进制负载，最大 1024 字节，超长数据报整包丢弃。
// 心跳格式：HEARTBEAT seq=<序号> uptime_ms=<开机毫秒>\n。
// 普通断网后重新监听并继续向原对端发送心跳；stop 清除对端和序号。
// Socket 创建、绑定、recvfrom/sendto 和独立任务的结构参考 Espressif 官方示例：
// https://github.com/espressif/esp-idf/blob/v5.5.4/examples/protocols/sockets/udp_server/main/udp_server.c
// 此处增加非阻塞收发、心跳、断网恢复和协作退出，不从 Shell 强制删除任务或关闭 Socket。

namespace {

constexpr char log_tag[] = "UDP_TEST";
constexpr int64_t heartbeat_interval_ms = 1000;
constexpr int64_t bind_retry_interval_ms = 1000;
constexpr size_t max_payload_size = 1024;
constexpr size_t header_capacity = 128;
constexpr unsigned packets_per_poll = 4;
constexpr uint16_t default_port = 5000;

// 命令由同一个 Console REPL 任务串行执行；只有它访问 worker_task。
TaskHandle_t worker_task = nullptr;
StaticSemaphore_t stopped_storage;
SemaphoreHandle_t stopped = nullptr;

// 以下状态只在工作任务运行时由它访问；Shell 仅在创建前设置端口。
// 停止确认前，Shell 不读写 Socket、对端、序号或缓冲区。
uint16_t local_port = 0;
int udp_socket = -1;
IPAddress bound_ip;
sockaddr_in heartbeat_peer{};
bool peer_known = false;
uint32_t heartbeat_sequence = 0;
uint32_t echo_sequence = 0;
int64_t next_heartbeat_ms = 0;
int64_t next_bind_attempt_ms = 0;

// 多接收一个字节即可识别超长数据报，避免将截断数据当作完整消息回环。
uint8_t receive_buffer[max_payload_size + 1];
char response_buffer[header_capacity + max_payload_size];

int64_t UptimeMs()
{
    return esp_timer_get_time() / 1000;
}

void CloseSocket()
{
    if (udp_socket >= 0)
    {
        close(udp_socket);
        udp_socket = -1;
    }
    bound_ip = IPAddress(0, 0, 0, 0);
}

bool OpenSocket(IPAddress ip)
{
    const int candidate = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (candidate < 0)
    {
        ESP_LOGW(log_tag, "Socket creation failed: errno=%d", errno);
        return false;
    }

    const int flags = fcntl(candidate, F_GETFL, 0);
    if (flags < 0 || fcntl(candidate, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        ESP_LOGW(log_tag, "Nonblocking setup failed: errno=%d", errno);
        close(candidate);
        return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(local_port);
    address.sin_addr.s_addr = static_cast<uint32_t>(ip);
    if (bind(candidate, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0)
    {
        ESP_LOGW(log_tag, "Bind failed: errno=%d", errno);
        close(candidate);
        return false;
    }

    udp_socket = candidate;
    bound_ip = ip;
    ESP_LOGI(log_tag, "Listening on %s:%u", ip.toString().c_str(),
             static_cast<unsigned>(local_port));
    return true;
}

void SendPacket(const sockaddr_in &destination, const char *data, size_t size)
{
    const ssize_t sent = sendto(
        udp_socket, data, size, 0,
        reinterpret_cast<const sockaddr *>(&destination), sizeof(destination));
    if (sent < 0)
    {
        ESP_LOGW(log_tag, "Send failed: errno=%d", errno);
    }
}

void Poll()
{
    const int64_t now_ms = UptimeMs();
    const IPAddress current_ip = ETH.localIP();
    if (!ETH.started() || !ETH.connected() || !ETH.hasIP() ||
        static_cast<uint32_t>(current_ip) == 0)
    {
        CloseSocket();
        next_bind_attempt_ms = 0;
        next_heartbeat_ms = now_ms + heartbeat_interval_ms;
        return;
    }

    // DHCP 可能在不重启设备的情况下分配新地址，重新绑定到当前有线地址。
    if (udp_socket >= 0 && bound_ip != current_ip)
    {
        CloseSocket();
        next_bind_attempt_ms = 0;
    }
    if (udp_socket < 0)
    {
        if (now_ms < next_bind_attempt_ms)
        {
            return;
        }
        next_bind_attempt_ms = now_ms + bind_retry_interval_ms;
        if (!OpenSocket(current_ip))
        {
            return;
        }
        next_heartbeat_ms = now_ms + heartbeat_interval_ms;
    }

    for (unsigned count = 0; count < packets_per_poll; ++count)
    {
        sockaddr_in sender{};
        socklen_t sender_size = sizeof(sender);
        const ssize_t received = recvfrom(
            udp_socket, receive_buffer, sizeof(receive_buffer), 0,
            reinterpret_cast<sockaddr *>(&sender), &sender_size);
        if (received < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            {
                break;
            }
            ESP_LOGW(log_tag, "Receive failed: errno=%d", errno);
            CloseSocket();
            next_bind_attempt_ms = UptimeMs() + bind_retry_interval_ms;
            return;
        }

        const int64_t received_ms = UptimeMs();
        if (static_cast<size_t>(received) > max_payload_size)
        {
            ESP_LOGW(log_tag, "Dropped datagram larger than %u bytes",
                     static_cast<unsigned>(max_payload_size));
            continue;
        }
        if (sender.sin_family != AF_INET || sender.sin_port == 0)
        {
            continue;
        }
        if (!peer_known)
        {
            heartbeat_peer = sender;
            peer_known = true;
            next_heartbeat_ms = received_ms + heartbeat_interval_ms;
            ESP_LOGI(log_tag, "Heartbeat peer: %s:%u",
                     IPAddress(sender.sin_addr.s_addr).toString().c_str(),
                     static_cast<unsigned>(ntohs(sender.sin_port)));
        }

        const int header_size = snprintf(
            response_buffer, header_capacity,
            "ECHO seq=%" PRIu32 " rx_ms=%" PRIi64 " len=%u\n",
            ++echo_sequence, received_ms, static_cast<unsigned>(received));
        if (header_size < 0 || static_cast<size_t>(header_size) >= header_capacity)
        {
            continue;
        }
        memcpy(response_buffer + header_size, receive_buffer, static_cast<size_t>(received));
        SendPacket(sender, response_buffer,
                   static_cast<size_t>(header_size) + static_cast<size_t>(received));
    }

    const int64_t heartbeat_ms = UptimeMs();
    if (peer_known && heartbeat_ms >= next_heartbeat_ms)
    {
        next_heartbeat_ms = heartbeat_ms + heartbeat_interval_ms;
        const int size = snprintf(
            response_buffer, header_capacity,
            "HEARTBEAT seq=%" PRIu32 " uptime_ms=%" PRIi64 "\n",
            ++heartbeat_sequence, heartbeat_ms);
        if (size > 0 && static_cast<size_t>(size) < header_capacity)
        {
            SendPacket(heartbeat_peer, response_buffer, static_cast<size_t>(size));
        }
    }
}

void ResetTestState()
{
    CloseSocket();
    local_port = 0;
    heartbeat_peer = sockaddr_in{};
    peer_known = false;
    heartbeat_sequence = 0;
    echo_sequence = 0;
    next_heartbeat_ms = 0;
    next_bind_attempt_ms = 0;
}

void TestTask(void *)
{
    // 10 ms 等待同时用于让出 CPU 和接收停止通知；Socket 收发均为非阻塞。
    // 出错时 Poll() 保留任务并重试，只有 stop 通知会让本任务退出。
    while (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10)) == 0)
    {
        Poll();
    }

    ResetTestState();
    // 发出确认后不再访问测试状态，Shell 才能安全地重新开启测试。
    xSemaphoreGive(stopped);
    vTaskDelete(nullptr);
}

int StartTest(uint16_t port)
{
    if (worker_task != nullptr)
    {
        printf("UDP test already running on port %u; stop it before restarting.\n",
               static_cast<unsigned>(local_port));
        return local_port == port ? 0 : 1;
    }

    if (stopped == nullptr)
    {
        stopped = xSemaphoreCreateBinaryStatic(&stopped_storage);
        if (stopped == nullptr)
        {
            ESP_LOGE(log_tag, "Stop semaphore initialization failed");
            return 1;
        }
    }

    if (!ETH.started())
    {
        ESP_LOGE(log_tag, "Ethernet is not started; check the startup log");
        return 1;
    }

    ResetTestState();
    local_port = port;
    xSemaphoreTake(stopped, 0);
    if (xTaskCreate(TestTask, "udp_test", 4096, nullptr, 2, &worker_task) != pdPASS)
    {
        worker_task = nullptr;
        local_port = 0;
        ESP_LOGE(log_tag, "UDP test task creation failed");
        return 1;
    }

    printf("UDP test started on port %u. Send a datagram to register the heartbeat peer.\n",
           static_cast<unsigned>(port));
    return 0;
}

int StopTest()
{
    if (worker_task == nullptr)
    {
        printf("UDP test is already stopped.\n");
        return 0;
    }

    // 工作任务只会在收到本通知后退出，因此这里的任务句柄仍然有效。
    xTaskNotifyGive(worker_task);
    xSemaphoreTake(stopped, portMAX_DELAY);
    worker_task = nullptr;
    printf("UDP test stopped.\n");
    return 0;
}

int HandleCommand(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "stop") == 0)
    {
        return StopTest();
    }
    if ((argc == 2 || argc == 3) && strcmp(argv[1], "start") == 0)
    {
        uint16_t port = default_port;
        if (argc == 3)
        {
            char *end = nullptr;
            errno = 0;
            const unsigned long value = strtoul(argv[2], &end, 10);
            if (argv[2][0] == '-' || errno != 0 || end == argv[2] ||
                *end != '\0' || value == 0 || value > 65535)
            {
                printf("Port must be an integer from 1 to 65535.\n");
                return 1;
            }
            port = static_cast<uint16_t>(value);
        }
        return StartTest(port);
    }

    printf("Usage: udp_test start [port] | udp_test stop\n");
    return 1;
}

} // namespace

namespace ind::hardware::w5500::_udp_test {

esp_err_t RegisterUdpTest()
{
    static IndShellCommand command = {
        "udp_test",
        "Start or stop the W5500 UDP echo and heartbeat test",
        "start [port] | stop",
        HandleCommand,
        nullptr
    };
    return ind_shell_register_command(&command);
}

} // namespace ind::hardware::w5500::_udp_test
