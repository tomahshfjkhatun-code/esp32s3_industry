#ifndef IND_HARDWARE_TWAI_ESP32_TWAI_INTERNAL_H
#define IND_HARDWARE_TWAI_ESP32_TWAI_INTERNAL_H

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * 使用契约（唯一控制器，单例，无 close）
 * ------------------------------------
 * 生命周期：开机由生命周期所有者调用一次 esp32_twai_open()，之后一直运行，无关闭。
 *
 * 线程模型（两个专用任务，本模块不创建任务）：
 *   TX 任务：固定到 open 所在核，循环： if (有数据) esp32_twai_send(&frame, timeout_ms);
 *   RX 任务：可任意核，但必须独立常驻循环，任何时候都不能等待 TX 返回：
 *            for (;;) esp32_twai_receive(&frame, timeout_ms);
 *   bus-off 恢复与状态通知都在 RX 任务的 receive 内部完成，RX 停则无法恢复。
 *
 * 缓冲区约定：
 *   send 借用 frame->buffer（不拷贝），返回前调用者不得修改/复用该缓冲；
 *   receive 要求调用者提供固定 8 字节 buffer，且 buffer_len 必须为 8。
 *
 * 最小示例：
 *   uint8_t tx_buf[8], rx_buf[8];
 *   twai_frame_t tx = {.buffer = tx_buf};                 // send 前填充 header/dlc/buffer_len
 *   twai_frame_t rx = {.buffer = rx_buf, .buffer_len = 8};
 *
 *   void tx_task(void *arg) {                              // 固定到 open 的核
 *       for (;;) { if (有数据) { 填 tx; esp32_twai_send(&tx, 500); } }
 *   }
 *   void rx_task(void *arg) {                              // 任意核，独立常驻
 *       for (;;) { if (esp32_twai_receive(&rx, 100) == ESP_OK) 处理(&rx); }
 *   }
 *   启动顺序：esp32_twai_open(&config, on_error, ctx) → 创建 tx/rx 任务 → 运行。
 */

/* RX 静态邮箱容量，单位为帧；默认 8，可通过全工程编译宏覆盖，必须大于 0。 */
#ifndef ESP32_TWAI_RX_MAILBOX_DEPTH
#define ESP32_TWAI_RX_MAILBOX_DEPTH 8u
#endif

/* open 起累计；发送超时属于调用结果，不等同于确认发送失败。 */
typedef struct {
    uint64_t sent;             /* 已提交事务中确认发送成功的次数。 */
    uint64_t terminal_failed;  /* 已提交事务中确认失败或被 bus-off 中止的次数。 */
    uint64_t tx_timeout;       /* 发送调用到期次数；可与实际成功/失败同时计数。 */
    uint64_t rx_dropped;       /* SDK 读取失败、格式无效或邮箱满导致的丢帧次数。 */
} Esp32TwaiStatistics;

/* 错误状态变化通知，可合并中间变化；只由唯一 RX 任务在 receive 中
 * 分发，锁外执行。必须快速返回，禁止重入收发；允许查询。快照仅回调期间有效。
 * context 指向的对象在设备运行期间必须有效，本层只保存并原样传回指针。 */
typedef void (*Esp32TwaiErrorCallback)(void *context, twai_error_state_t old_state,
                                     twai_error_state_t new_state,
                                     const Esp32TwaiStatistics *statistics);

#ifdef __cplusplus
extern "C" {
#endif

/* 本模块为单例，同步打开唯一控制器，不创建任务。配置复制一次；经典 CAN，tx_queue_depth=1，
 * fail_retry_cnt=-1，禁止 listen-only。open 与唯一 TX 任务必须固定在同一 CPU，
 * RX 可在另一 CPU。所有收发和查询只能在 open 成功返回后开始。
 * RX 必须持续独立循环调用 receive，空闲或 TX 阻塞期间也不能暂停、等待 TX 返回。
 * TX 无待发送数据时阻塞等待上层数据；拿到后调用 send，返回后再处理下一项。
 * 成功后不支持重复 open，无 close 接口；失败且完整回滚后可重试 open。
 * GPIO、位时序等通用配置由 SDK 校验，本层额外检查上述发送队列、重试和模式约定。 */
esp_err_t esp32_twai_open(const twai_onchip_node_config_t *config,
                          Esp32TwaiErrorCallback on_error, void *error_context);

/* 严格同步发送：-1 无限等待；1..60000 为完成确认和恢复收尾的期限（毫秒）；0 拒绝。
 * 直接使用 twai_frame_t，仅支持经典 CAN：DLC 范围 0..8，ID 合法范围由调用者保证。
 * 数据帧 buffer_len 为有效数据长度（0..8），非零时 buffer 必须有效。
 * DLC 非零时由 SDK 检查与长度一致；DLC 为 0 时 SDK 根据 buffer_len 推算。
 * 远程帧 buffer_len 必须为 0，不发送 buffer；发送忽略 timestamp/trigger_time。
 * 调用者可用固定 uint8_t buffer[8] 提供存储，本层不分配数据缓冲。
 * 毫秒参数只转换一次，剩余预算由 FreeRTOS 管理；精度受 tick 粒度影响。
 * 超时只标记结果，仍阻塞至 SDK 结束事务，实际阻塞时间可能超过期限甚至无限。
 * 仅适配帧头，数据直接借用 frame->buffer；整个调用期间帧不得被修改或复用。
 * 所有返回路径均不再借用输入内存；不存在留给下次 send 结算的在途帧。
 * 最多提交一次。TX 不发起恢复或分发用户状态通知，恢复由常驻 RX 任务负责。
 * 调用时已 bus-off：不提交，立即返回 INVALID_STATE；SDK 拒收也立即原样返回。
 * 已提交后遇到 bus-off：等待 RX 恢复并确认 SDK 结束引用，再退出，不重新提交。
 * ESP_OK：确认成功；ESP_FAIL：确认失败或已提交帧被 bus-off 中止。
 * ESP_ERR_TIMEOUT：期限内未完成收尾；实际结果仍计入 sent/terminal_failed。
 * 其余 SDK 错误原样返回；参数错误 INVALID_ARG；未打开/核不符 INVALID_STATE。
 * 由唯一 TX 线程调用，必须与 open 同核；禁止并发和 ISR/错误回调中调用。
 * 单发送者由调用者保证，本封装不做同方向并发检查。 */
esp_err_t esp32_twai_send(const twai_frame_t *frame, int timeout_ms);

/* -1 无限等；0 不等待；1..60000 为取帧预算。
 * 毫秒参数只转换一次，剩余预算由 FreeRTOS 管理；精度受 tick 粒度影响。
 * frame->buffer 必须指向调用者提供的固定 8 字节存储，buffer_len 必须为 8。
 * 成功时写入 header 和 8 字节数据；header.dlc 为实际长度或远程请求长度。
 * 未使用的数据清零，远程帧数据全零；timestamp 为本机微秒。
 * 保留 buffer 指针和 buffer_len=8，便于连续接收；实际长度读取 header.dlc。
 * 超时返回不修改帧描述符及数据。
 * 由唯一 RX 线程调用，禁止并发或错误回调重入；本封装不做同方向并发检查。
 * 从深度为 ESP32_TWAI_RX_MAILBOX_DEPTH 的静态邮箱取帧，按到达顺序接收。
 * 邮箱按值保存私有 TwaiFrame 的帧头和内嵌数据，取帧时复制给调用者，无需归还。
 * 邮箱满时丢新帧。状态变化也会唤醒接收等待。
 * 仅本函数发起 bus-off 恢复并分发用户状态通知，不依赖新报文到达。
 * RX 必须常驻独立循环调用，不能等待 TX 返回；无内部恢复任务。 */
esp_err_t esp32_twai_receive(twai_frame_t *frame, int timeout_ms);

/* 统一查询：status/record 直接读取 SDK，statistics 读取本层加锁统计快照。
 * 三个输出均可为 NULL，但至少提供一个；调用者提供独立输出对象，本层不检查地址重叠。
 * 只查询 statistics 时不调用 SDK。三类信息依次读取，不保证同一时刻快照。
 * 本函数只查询，不分发用户通知；未打开返回 INVALID_STATE，SDK 错误原样返回。 */
esp_err_t esp32_twai_get_info(twai_node_status_t *status, twai_node_record_t *record,
                             Esp32TwaiStatistics *statistics);

#ifdef __cplusplus
}
#endif

#endif
