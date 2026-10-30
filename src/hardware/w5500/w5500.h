#pragma once

#include <IPAddress.h>

namespace ind::hardware::w5500 {

// 仅在 setup() 中调用。每次启动只尝试初始化一次，不等待网线连接或 DHCP。
// 成功后收集 udp_test 命令，必须先于 shell::Initialize() 调用。
// 默认使用静态 IPv4；ip 为 0.0.0.0 时使用 DHCP，忽略其余地址参数。
// 重复调用返回第一次初始化的结果，不重新应用配置；修改配置后需重启。
// 返回 true 仅表示驱动启动及地址模式配置成功，不代表链路或远端已可用。
// 本模块独占全局 ETH 和所用的 FSPI 总线，Wi-Fi 由应用层管理。
//
// 后续直接通过 ETH.connected() && ETH.hasIP() 判断接口是否可通信，
// 通过 ETH.localIP() 读取地址；静态 IP 在拔线后仍可保留，不能单独判断链路。
// 业务可在初始化前通过 Network.onEvent() 注册自己的网络事件回调：
// ETH_CONNECTED / ETH_GOT_IP 表示链路或地址就绪，ETH_DISCONNECTED /
// ETH_LOST_IP / ETH_STOP 表示链路断开、地址丢失或驱动停止
// （事件名均带 ARDUINO_EVENT_ 前缀）。普通拔插无需重新初始化驱动。
// 回调在 Arduino 网络事件任务中执行，仅做线程安全的状态更新或任务通知；
// Socket 的关闭和重建交给业务任务，初始化成功后还应查询一次当前接口状态。
bool Initialize(
    IPAddress ip = IPAddress(192, 168, 3, 50),
    IPAddress gateway = IPAddress(192, 168, 3, 1),
    IPAddress subnet = IPAddress(255, 255, 255, 0),
    IPAddress dns = IPAddress(192, 168, 3, 1));

} // namespace ind::hardware::w5500
