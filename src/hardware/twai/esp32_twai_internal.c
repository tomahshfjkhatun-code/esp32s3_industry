#include "esp32_twai_internal.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>

_Static_assert(TWAI_FRAME_MAX_LEN == 8 && TWAI_FRAME_MAX_DLC == 8, "Classic TWAI requires 8 byte frames");
_Static_assert(ESP32_TWAI_RX_MAILBOX_DEPTH > 0, "RX mailbox depth must be positive");

/* 唯一控制器的共用状态；收发资源和用户通知各自保存在下面的私有对象中。 */
typedef struct {
    twai_node_handle_t node;         /* SDK 控制器句柄；创建/删除时在 lock 下更新。 */
    twai_onchip_node_config_t config; /* open 的配置副本，保留供以后内部重建使用。 */
    enum { DEVICE_CLOSED, DEVICE_RUNNING } state; /* 生命周期；控制业务调用和 ISR 处理。 */
    BaseType_t core;                 /* open 所在 CPU；TX 必须与节点中断同核。 */
    portMUX_TYPE lock;               /* 保护共用状态、统计、TX 结果、RX 恢复标志和待通知状态。 */
    twai_error_state_t error_state;  /* 当前总线错误状态，由状态 ISR 更新。 */
    uint64_t bus_off_generation;     /* bus-off 历史序号；ISR 递增、send 比较基准，不属于统计。 */
    Esp32TwaiStatistics statistics;  /* 成功、失败、超时、RX 丢帧四项累计统计。 */
} Esp32TwaiDevice;

/* RX 邮箱元素按值拥有帧头和数据，不保存指向外部存储的指针。 */
typedef struct {
    twai_frame_header_t header;      /* SDK 解码的帧头。 */
    uint8_t buffer[8];              /* 内嵌经典 CAN 数据，随队列元素一起复制。 */
} TwaiFrame;

/* RX ISR 写邮箱，唯一 RX 任务取帧；队列/信号量内部完成同步。 */
typedef struct {
    QueueHandle_t mailbox;          /* 宏指定深度的接收 FIFO 邮箱，元素为 TwaiFrame。 */
    SemaphoreHandle_t ready;        /* 帧到达或状态变化时唤醒 receive。 */
    StaticQueue_t _rx_mailbox_storage; /* 邮箱的静态控制对象存储。 */
    uint8_t _rx_mailbox_buffer[ESP32_TWAI_RX_MAILBOX_DEPTH * sizeof(TwaiFrame)]; /* 邮箱的完整帧存储。 */
    StaticSemaphore_t _rx_ready_storage; /* ready 信号量的静态控制对象存储。 */
    bool recovering;                /* 当前恢复请求已登记；失败或状态 ISR 清零，受 s_device.lock 保护。 */
} Esp32TwaiRx;

typedef struct {
    SemaphoreHandle_t done;         /* TX 完成或状态变化时唤醒 send。 */
    enum { TX_UNKNOWN, TX_SUCCESS, TX_FAILED } result; /* ISR 写、send 读；受 s_device.lock 保护。 */
    StaticSemaphore_t _tx_done_storage; /* done 信号量的静态控制对象存储。 */
} Esp32TwaiTx;

/* ISR 留下通知，唯一 RX 任务消费；待通知状态受 s_device.lock 保护。 */
typedef struct {
    Esp32TwaiErrorCallback on_error; /* open 注册的用户回调，NULL 表示不通知。 */
    void *context;                  /* 用户上下文，只保存并原样传给回调。 */
    twai_error_state_t pending_old; /* 最近待通知转换的旧状态；等于 pending_new 时没有通知。 */
    twai_error_state_t pending_new; /* 最近待通知转换的新状态；ISR 更新，分发后消费。 */
} Esp32TwaiStateNotice;

/* 本模块持有唯一实例；四份私有静态存储共同组成此单例，不对外传递设备句柄。 */
static DRAM_ATTR Esp32TwaiDevice s_device = {
    .state = DEVICE_CLOSED, .lock = portMUX_INITIALIZER_UNLOCKED
};
static DRAM_ATTR Esp32TwaiRx s_rx;
static DRAM_ATTR Esp32TwaiTx s_tx;
static DRAM_ATTR Esp32TwaiStateNotice s_state_notice;

/* 消费 SDK 状态回调留下的 old/new，在任务上下文通知用户总线状态变化（含恢复）。
 * 没有待通知时直接返回；有通知时取出状态/统计快照，解锁后调用用户 on_error。
 * 仅由唯一 RX 任务调用，禁止回调重入 receive；回调查询不会触发通知。 */
static void notify_pending_state_changes(void)
{
    if (s_state_notice.on_error == NULL) return;
    for (;;) {
        twai_error_state_t old_state, new_state;
        Esp32TwaiStatistics statistics;
        bool pending;
        portENTER_CRITICAL(&s_device.lock);
        pending = s_device.state == DEVICE_RUNNING && s_state_notice.pending_old != s_state_notice.pending_new;
        if (!pending) {
            portEXIT_CRITICAL(&s_device.lock);
            return;
        }
        old_state = s_state_notice.pending_old;
        new_state = s_state_notice.pending_new;
        statistics = s_device.statistics;
        s_state_notice.pending_old = s_state_notice.pending_new; /* 两者相等表示没有待通知的变化。 */
        portEXIT_CRITICAL(&s_device.lock);
        s_state_notice.on_error(s_state_notice.context, old_state, new_state, &statistics);
    }
}

/* SDK TX 完成中断：登记实际发送结果，唤醒等待中的 send；不直接结算统计。 */
static bool IRAM_ATTR twai_tx_done_callback(twai_node_handle_t node,
                                            const twai_tx_done_event_data_t *event, void *context)
{
    (void)node;
    (void)context; /* SDK 固定回调签名；单例不使用 user_data。 */
    portENTER_CRITICAL_ISR(&s_device.lock);
    if (s_device.state != DEVICE_RUNNING) {
        portEXIT_CRITICAL_ISR(&s_device.lock);
        return false;
    }
    /* 唯一发送者，调用前复位结果、返回前排空事务；不需要另存发送活动标志。 */
    if (event != NULL && s_tx.result == TX_UNKNOWN) {
        s_tx.result = event->is_tx_success ? TX_SUCCESS : TX_FAILED;
    }
    portEXIT_CRITICAL_ISR(&s_device.lock);
    BaseType_t woken = pdFALSE;
    (void)xSemaphoreGiveFromISR(s_tx.done, &woken);
    return woken == pdTRUE;
}
/* SDK RX 中断：临时描述符适配 SDK，完整帧按值入邮箱。 */
static bool IRAM_ATTR twai_rx_done_callback(twai_node_handle_t node,
                                            const twai_rx_done_event_data_t *event, void *context)
{
    (void)context; /* SDK 固定回调签名；单例不使用 user_data。 */
    (void)event;
    portENTER_CRITICAL_ISR(&s_device.lock);
    if (s_device.state != DEVICE_RUNNING) {
        portEXIT_CRITICAL_ISR(&s_device.lock);
        return false;
    }
    portEXIT_CRITICAL_ISR(&s_device.lock);
    TwaiFrame received = {0};
    twai_frame_t native = {.buffer = received.buffer, .buffer_len = sizeof(received.buffer)};
    const esp_err_t result = twai_node_receive_from_isr(node, &native);
    received.header = native.header;
    /* SDK 解码帧头；只过滤固定邮箱无法表达的 DLC，不重复检查 ID 或标志。 */
    const bool valid = result == ESP_OK && received.header.dlc <= sizeof(received.buffer);
    received.header.timestamp = (uint64_t)esp_timer_get_time();
    BaseType_t woken = pdFALSE;
    const bool queued = valid && xQueueSendFromISR(s_rx.mailbox, &received, &woken) == pdTRUE;
    portENTER_CRITICAL_ISR(&s_device.lock);
    if (!queued) ++s_device.statistics.rx_dropped;
    portEXIT_CRITICAL_ISR(&s_device.lock);
    (void)xSemaphoreGiveFromISR(s_rx.ready, &woken);
    return woken == pdTRUE;
}
/* SDK 状态中断：更新总线状态和 bus-off 序号，留下任务通知，唤醒收发。 */
static bool IRAM_ATTR twai_state_change_callback(twai_node_handle_t node,
                                                 const twai_state_change_event_data_t *event, void *context)
{
    (void)node;
    (void)context; /* SDK 固定回调签名；单例不使用 user_data。 */
    if (event == NULL) return false;
    portENTER_CRITICAL_ISR(&s_device.lock);
    if (s_device.state != DEVICE_RUNNING) {
        portEXIT_CRITICAL_ISR(&s_device.lock);
        return false;
    }
    /* 保留最近一次状态转换；bus-off 序号供 send 判断本次是否被中止。 */
    s_device.error_state = event->new_sta;
    if (event->old_sta != event->new_sta) {
        s_state_notice.pending_old = event->old_sta;
        s_state_notice.pending_new = event->new_sta;
        if (event->new_sta == TWAI_ERROR_BUS_OFF) {
            ++s_device.bus_off_generation;
            s_rx.recovering = false;
        } else if (event->old_sta == TWAI_ERROR_BUS_OFF) {
            s_rx.recovering = false;
        }
    }
    portEXIT_CRITICAL_ISR(&s_device.lock);
    BaseType_t woken = pdFALSE;
    (void)xSemaphoreGiveFromISR(s_rx.ready, &woken); /* 唤醒 RX 发起恢复并通知用户。 */
    (void)xSemaphoreGiveFromISR(s_tx.done, &woken);  /* 唤醒 TX 检查完成或恢复后的收尾。 */
    return woken == pdTRUE;
}

esp_err_t esp32_twai_open(const twai_onchip_node_config_t *config,
                          Esp32TwaiErrorCallback on_error, void *error_context)
{
    /* 只检查本封装的使用约定；GPIO、时序等硬件配置交给 SDK 校验。 */
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    if (config->tx_queue_depth != 1 || config->fail_retry_cnt != -1 || config->flags.enable_listen_only)
        return ESP_ERR_INVALID_ARG;
    if (xPortInIsrContext()) return ESP_ERR_INVALID_STATE;
    portENTER_CRITICAL(&s_device.lock);
    const bool busy = s_device.state != DEVICE_CLOSED || s_device.node != NULL ||
        s_rx.mailbox != NULL || s_rx.ready != NULL || s_tx.done != NULL;
    portEXIT_CRITICAL(&s_device.lock);
    if (busy) return ESP_ERR_INVALID_STATE;

    /* 保存配置和用户回调，记录中断所在核，初始化状态与统计。 */
    s_device.config = *config;
    s_state_notice.on_error = on_error;
    s_state_notice.context = error_context;
    s_device.core = xPortGetCoreID();
    s_device.error_state = TWAI_ERROR_ACTIVE;
    s_device.bus_off_generation = 0;
    s_rx.recovering = false;
    s_tx.result = TX_UNKNOWN;
    memset(&s_device.statistics, 0, sizeof(s_device.statistics));

    /* 创建静态 RX 邮箱和 RX/TX 信号量，在启用中断前准备好同步资源。 */
    s_rx.mailbox = xQueueCreateStatic(ESP32_TWAI_RX_MAILBOX_DEPTH, sizeof(TwaiFrame),
                                      s_rx._rx_mailbox_buffer, &s_rx._rx_mailbox_storage);
    s_rx.ready = xSemaphoreCreateBinaryStatic(&s_rx._rx_ready_storage);
    if (s_rx.ready != NULL) s_tx.done = xSemaphoreCreateBinaryStatic(&s_tx._tx_done_storage);

    /* 同步资源齐备后创建 SDK 节点，由 SDK 校验通用配置。 */
    esp_err_t result = s_rx.mailbox == NULL || s_tx.done == NULL ? ESP_ERR_NO_MEM : ESP_OK;
    twai_node_handle_t node = NULL;
    if (result == ESP_OK) {
        result = twai_new_node_onchip(&s_device.config, &node);
        if (result == ESP_OK) {
            portENTER_CRITICAL(&s_device.lock);
            s_device.node = node;
            portEXIT_CRITICAL(&s_device.lock);
        }
    }

    /* 单例回调直接访问模块存储，不需要通过 SDK user_data 传设备指针。 */
    if (result == ESP_OK) {
        const twai_event_callbacks_t callbacks = {
            .on_tx_done = twai_tx_done_callback, .on_rx_done = twai_rx_done_callback,
            .on_state_change = twai_state_change_callback
        };
        result = twai_node_register_event_callbacks(node, &callbacks, NULL);
    }

    /* 先置 RUNNING，再启用硬件，接住 enable 返回前的启动中断。 */
    if (result == ESP_OK) {
        portENTER_CRITICAL(&s_device.lock);
        s_device.state = DEVICE_RUNNING;
        portEXIT_CRITICAL(&s_device.lock);
        result = twai_node_enable(node);
    }

    /* 失败时先置 CLOSED 并停用/删除节点，再清理同步对象。
     * 节点清理失败时保留资源，防止重复创建；完整回滚后才允许重试 open。 */
    if (result != ESP_OK) {
        portENTER_CRITICAL(&s_device.lock);
        s_device.state = DEVICE_CLOSED;
        portEXIT_CRITICAL(&s_device.lock);
        esp_err_t cleanup = ESP_OK;
        if (s_device.node != NULL) {
            cleanup = twai_node_disable(s_device.node);
            if (cleanup == ESP_ERR_INVALID_STATE) {
                /* 未启用或已 bus-off 的节点允许直接删除。 */
                twai_node_status_t status;
                cleanup = twai_node_get_info(s_device.node, &status, NULL);
                if (cleanup == ESP_OK && status.state != TWAI_ERROR_BUS_OFF) cleanup = ESP_ERR_INVALID_STATE;
            }
            if (cleanup == ESP_OK) {
                cleanup = twai_node_delete(s_device.node);
                if (cleanup == ESP_OK) {
                    portENTER_CRITICAL(&s_device.lock);
                    s_device.node = NULL;
                    portEXIT_CRITICAL(&s_device.lock);
                }
            }
        }
        if (cleanup == ESP_OK) {
            if (s_rx.mailbox != NULL) vQueueDelete(s_rx.mailbox);
            if (s_rx.ready != NULL) vSemaphoreDelete(s_rx.ready);
            if (s_tx.done != NULL) vSemaphoreDelete(s_tx.done);
            s_rx.mailbox = NULL;
            s_rx.ready = NULL;
            s_tx.done = NULL;
            s_state_notice.pending_old = s_state_notice.pending_new;
        }
    }

    /* 打开结果直接返回；节点清理失败时保留的资源继续阻止重复打开。 */
    return result;
}

esp_err_t esp32_twai_send(const twai_frame_t *frame, int timeout_ms)
{
    /* 校验本层的经典 CAN/固定存储约定和调用上下文；通用 DLC/长度匹配交给 SDK。 */
    if (frame == NULL || timeout_ms == 0 || timeout_ms < -1 || timeout_ms > 60000)
        return ESP_ERR_INVALID_ARG;
    if (frame->header.fdf || frame->header.brs || frame->header.esi || frame->header.dlc > 8)
        return ESP_ERR_INVALID_ARG;
    if (frame->header.rtr ? frame->buffer_len != 0 :
        (frame->buffer_len > 8 || (frame->buffer_len != 0 && frame->buffer == NULL) ||
         (frame->header.dlc != 0 && frame->buffer_len == 0)))
        return ESP_ERR_INVALID_ARG;
    if (xPortInIsrContext() || xPortGetCoreID() != s_device.core) return ESP_ERR_INVALID_STATE;

    /* 毫秒参数只转换一次，完成与收尾共用剩余预算；当前 SDK 为 1ms/tick。 */
    TickType_t remaining_ticks = timeout_ms < 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    TimeOut_t timeout_state;
    vTaskSetTimeOutState(&timeout_state);

    portENTER_CRITICAL(&s_device.lock);
    /* 尚未提交时，未打开或 bus-off 直接拒绝；恢复由常驻 RX 任务负责。 */
    if (s_device.state != DEVICE_RUNNING || s_device.error_state == TWAI_ERROR_BUS_OFF) {
        portEXIT_CRITICAL(&s_device.lock);
        return ESP_ERR_INVALID_STATE;
    }
    portEXIT_CRITICAL(&s_device.lock);

    /* 准备本次 SDK 描述符，忽略延迟发送设置，数据直接借用调用者的固定数组。
     * 清理上次状态通知留下的令牌；上次 send 已收尾，不存在遗留事务。 */
    twai_frame_t native = {
        .header = frame->header,
        .buffer = frame->buffer,
        .buffer_len = frame->buffer_len
    };
    native.header.trigger_time = 0;
    (void)xSemaphoreTake(s_tx.done, 0); /* 清掉上次状态通知留下的令牌，不结算旧事务。 */

    /* 记录本次 bus-off 序号基准，提交前复位发送结果。
     * 完成中断可能先于 transmit 返回，因此先复位再提交。 */
    portENTER_CRITICAL(&s_device.lock);
    const uint64_t bus_off_at_start = s_device.bus_off_generation;
    s_tx.result = TX_UNKNOWN;
    portEXIT_CRITICAL(&s_device.lock);

    /* 最多提交一次；SDK 拒收时不借用输入，立即原样返回，不等待恢复。 */
    esp_err_t result = twai_node_transmit(s_device.node, &native, 0);
    if (result != ESP_OK) return result;

    /* 已提交后只等待完成或 RX 恢复后的收尾，不发起恢复，也不重新提交。 */
    bool timed_out = false;
    for (;;) {
        portENTER_CRITICAL(&s_device.lock);
        /* 私有 bus-off 序号在运行期不清零；恢复后也保留本次发送的历史。 */
        const bool terminal = s_tx.result != TX_UNKNOWN ||
            s_device.bus_off_generation != bus_off_at_start;
        const bool can_finish = terminal && s_device.error_state != TWAI_ERROR_BUS_OFF;
        portEXIT_CRITICAL(&s_device.lock);

        /* 已收到完成结果或发生过 bus-off，且当前不处于 bus-off 时，才确认 SDK 空闲。
         * wait_all_done 成功只表示事务结束，发送是否成功仍以 TX 回调结果为准。 */
        const esp_err_t waited = can_finish ? twai_node_transmit_wait_all_done(s_device.node, 0) : ESP_ERR_TIMEOUT;

        /* FreeRTOS 扣减剩余预算；portMAX_DELAY 表示无限等待。 */
        const bool budget_expired = !timed_out &&
            xTaskCheckForTimeOut(&timeout_state, &remaining_ticks) == pdTRUE;
        portENTER_CRITICAL(&s_device.lock);
        /* 到期只标记超时并计数一次；仍继续收尾，不在 SDK 借用数据时返回。 */
        if (budget_expired) {
            timed_out = true;
            ++s_device.statistics.tx_timeout;
        }
        /* 重新确认 ISR 状态，结束后结算实际结果；实际成功/失败与调用超时分别记录。 */
        const bool finished = can_finish && waited == ESP_OK && s_device.error_state != TWAI_ERROR_BUS_OFF;
        if (finished) {
            const bool success = s_tx.result == TX_SUCCESS;
            /* 每个已提交事务只结算一次，包括没有完成回调的 bus-off 中止。 */
            if (success) ++s_device.statistics.sent;
            else ++s_device.statistics.terminal_failed;
            s_tx.result = TX_UNKNOWN;
            result = success ? ESP_OK : ESP_FAIL;
        }
        portEXIT_CRITICAL(&s_device.lock);
        if (finished) {
            if (timed_out) result = ESP_ERR_TIMEOUT;
            break;
        }

        /* 等 TX 完成或总线状态通知；到期后仍等收尾。TX 与 SDK ISR 同核，
         * 被中断唤醒时 ISR 已结束，SDK 不会留下需要定时轮询的中断收尾。 */
        const TickType_t ticks = timed_out ? portMAX_DELAY : remaining_ticks;
        (void)xSemaphoreTake(s_tx.done, ticks);
    }

    /* 已提交事务已结束，返回后调用者可以复用输入内存。 */
    return result;
}

esp_err_t esp32_twai_receive(twai_frame_t *frame, int timeout_ms)
{
    /* 校验调用者的固定 8 字节存储、超时参数及上下文，禁止在 ISR 中阻塞接收。 */
    if (frame == NULL || frame->buffer == NULL || frame->buffer_len != 8 || timeout_ms < -1 || timeout_ms > 60000)
        return ESP_ERR_INVALID_ARG;
    if (xPortInIsrContext()) return ESP_ERR_INVALID_STATE;

    /* 毫秒参数只转换一次；0 不等待，-1 无限等；当前 SDK 为 1ms/tick。 */
    TickType_t remaining_ticks = timeout_ms < 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    TimeOut_t timeout_state;
    vTaskSetTimeOutState(&timeout_state);

    portENTER_CRITICAL(&s_device.lock);
    if (s_device.state != DEVICE_RUNNING) {
        portEXIT_CRITICAL(&s_device.lock);
        return ESP_ERR_INVALID_STATE;
    }
    portEXIT_CRITICAL(&s_device.lock);

    /* 接收任务负责恢复、状态通知和取帧；没有额外的内部工作任务。 */
    esp_err_t result = ESP_OK;
    for (;;) {
        /* 唯一 RX 任务负责恢复：每次 bus-off 只登记一次成功的异步请求。
         * SDK 调用在锁外；请求失败清标志，离开 bus-off 由状态 ISR 清标志。 */
        portENTER_CRITICAL(&s_device.lock);
        const bool start_recovery = s_device.error_state == TWAI_ERROR_BUS_OFF && !s_rx.recovering;
        if (start_recovery) s_rx.recovering = true;
        portEXIT_CRITICAL(&s_device.lock);
        const esp_err_t recovery_result = start_recovery ? twai_node_recover(s_device.node) : ESP_OK;
        if (recovery_result != ESP_OK) {
            portENTER_CRITICAL(&s_device.lock);
            s_rx.recovering = false;
            portEXIT_CRITICAL(&s_device.lock);
        }
        /* 用户状态通知也只在 RX 任务执行；即使没有新帧，也先恢复并通知。 */
        notify_pending_state_changes();
        /* 按值取出完整帧，复制给调用者；保留调用者的 buffer 指针和容量。 */
        TwaiFrame received;
        if (xQueueReceive(s_rx.mailbox, &received, 0) == pdTRUE) {
            frame->header = received.header;
            memcpy(frame->buffer, received.buffer, sizeof(received.buffer));
            break;
        }
        /* 邮箱为空则检查剩余预算；到期退出，调用者描述符及数据保持不变。 */
        if (xTaskCheckForTimeOut(&timeout_state, &remaining_ticks) == pdTRUE) {
            result = ESP_ERR_TIMEOUT;
            break;
        }
        /* 等帧到达或状态变化唤醒；恢复请求失败时至多等 10ms 后重试。 */
        TickType_t ticks = remaining_ticks;
        if (recovery_result != ESP_OK) {
            const TickType_t retry_ticks = pdMS_TO_TICKS(10);
            if (ticks > retry_ticks) ticks = retry_ticks;
        }
        (void)xSemaphoreTake(s_rx.ready, ticks);
    }
    /* 返回取帧结果前，再分发尚未处理的状态变化。 */
    notify_pending_state_changes();
    return result;
}

esp_err_t esp32_twai_get_info(twai_node_status_t *status, twai_node_record_t *record,
                             Esp32TwaiStatistics *statistics)
{
    if (status == NULL && record == NULL && statistics == NULL) return ESP_ERR_INVALID_ARG;
    if (xPortInIsrContext()) return ESP_ERR_INVALID_STATE;
    portENTER_CRITICAL(&s_device.lock);
    const bool running = s_device.state == DEVICE_RUNNING;
    portEXIT_CRITICAL(&s_device.lock);
    if (!running) return ESP_ERR_INVALID_STATE;
    /* 仅请求补充统计时跳过 SDK，三个输出是依次读取的快照。 */
    const esp_err_t result = status != NULL || record != NULL ?
        twai_node_get_info(s_device.node, status, record) : ESP_OK;
    if (result == ESP_OK && statistics != NULL) {
        portENTER_CRITICAL(&s_device.lock);
        *statistics = s_device.statistics;
        portEXIT_CRITICAL(&s_device.lock);
    }
    return result;
}
