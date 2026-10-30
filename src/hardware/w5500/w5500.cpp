#include "w5500.h"

#include "../../log/log.h"

#include <ETH.h>
#include <SPI.h>
#include <esp_err.h>

namespace ind::hardware::w5500 {
// 实现文件之间的内部接口，不在公开头文件中声明。
namespace _udp_test {

esp_err_t RegisterUdpTest();

} // namespace _udp_test

namespace {

constexpr char log_tag[] = "W5500";
constexpr char hostname[] = "esp32s3-industry";

// 板载连接见 docs/hardware_interface.md，全部显式指定。
constexpr int pin_rst = 8;
constexpr int pin_irq = 9;
constexpr int pin_mosi = 10;
constexpr int pin_miso = 11;
constexpr int pin_sck = 12;
constexpr int pin_cs = 13;
constexpr int phy_address = 1;
constexpr uint8_t spi_frequency_mhz = 20;

SPIClass spi(FSPI);
bool initialization_attempted = false;
bool initialized = false;

// Arduino 网络事件任务调用；不在这里访问业务 Socket 或等待网络恢复。
void OnNetworkEvent(arduino_event_id_t event)
{
    switch (event)
    {
    case ARDUINO_EVENT_ETH_START:
        if (!ETH.setHostname(hostname))
        {
            ESP_LOGW(log_tag, "Failed to set Ethernet hostname");
        }
        ESP_LOGI(log_tag, "Ethernet started");
        break;
    case ARDUINO_EVENT_ETH_CONNECTED:
        ESP_LOGI(log_tag, "Ethernet link up");
        break;
    case ARDUINO_EVENT_ETH_GOT_IP:
        ESP_LOGI(log_tag, "Ethernet IPv4: %s", ETH.localIP().toString().c_str());
        break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
        ESP_LOGW(log_tag, "Ethernet link down");
        break;
    case ARDUINO_EVENT_ETH_LOST_IP:
        ESP_LOGW(log_tag, "Ethernet lost IPv4");
        break;
    case ARDUINO_EVENT_ETH_STOP:
        ESP_LOGI(log_tag, "Ethernet stopped");
        break;
    default:
        break;
    }
}

} // namespace

bool Initialize(IPAddress ip, IPAddress gateway, IPAddress subnet, IPAddress dns)
{
    if (initialization_attempted)
    {
        return initialized;
    }
    initialization_attempted = true;

    if (ip.type() != IPv4)
    {
        ESP_LOGE(log_tag, "An IPv4 address is required");
        return false;
    }

    const bool use_dhcp = static_cast<uint32_t>(ip) == 0;
    if (!use_dhcp &&
        (gateway.type() != IPv4 || subnet.type() != IPv4 || dns.type() != IPv4 ||
         static_cast<uint32_t>(subnet) == 0))
    {
        ESP_LOGE(log_tag, "Static IPv4 requires IPv4 parameters and a nonzero subnet");
        return false;
    }

    const network_event_handle_t event_handle = Network.onEvent(OnNetworkEvent);
    spi.begin(pin_sck, pin_miso, pin_mosi, pin_cs);

    if (!ETH.begin(ETH_PHY_W5500, phy_address, pin_cs, pin_irq, pin_rst,
                   spi, spi_frequency_mhz))
    {
        ESP_LOGE(log_tag, "W5500 initialization failed");
        ETH.end();
        Network.removeEvent(event_handle);
        return false;
    }

    // begin() 创建网络接口后才能配置 IP。
    // DHCP 使用全零参数，避免混入静态网关、掩码或 DNS 默认值。
    const bool configured = use_dhcp ? ETH.config() : ETH.config(ip, gateway, subnet, dns);
    if (!configured)
    {
        ESP_LOGE(log_tag, "IPv4 configuration failed");
        ETH.end();
        Network.removeEvent(event_handle);
        return false;
    }

    initialized = true;
    if (use_dhcp)
    {
        ESP_LOGI(log_tag, "DHCP enabled; waiting for IPv4");
    }
    else
    {
        ESP_LOGI(log_tag, "Static IPv4 configured: IP=%s mask=%s gateway=%s DNS=%s",
                 ip.toString().c_str(), subnet.toString().c_str(),
                 gateway.toString().c_str(), dns.toString().c_str());
    }

    const esp_err_t command_error = _udp_test::RegisterUdpTest();
    if (command_error != ESP_OK)
    {
        ESP_LOGE(log_tag, "UDP test command registration failed: %s", esp_err_to_name(command_error));
    }
    return true;
}

} // namespace ind::hardware::w5500
