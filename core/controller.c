/* SPDX-License-Identifier: MIT */

#include "controller.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "camera_backend.h"
#include "pairing.h"

static const char *TAG = "CONTROLLER";

typedef enum {
    CTRL_SINGLE_PRESS = 0,
    CTRL_SHUTTER,
    CTRL_RECORD_START,
    CTRL_RECORD_STOP,
    CTRL_PRESET_NEXT,
    CTRL_SLEEP_WAKE,
    CTRL_POWER_OFF,
    CTRL_WAKE_BEACON,
    CTRL_SWITCH_PROTOCOL,
    CTRL_PAIRING_ENTER,
    CTRL_PAIRING_EXIT,
    CTRL_ACTION_COUNT,
} ctrl_action_t;

typedef struct {
    ctrl_action_t action;
    backend_id_t id;
} ctrl_msg_t;

static QueueHandle_t s_ctrl_queue = NULL;

/* 待处理动作位图：某类动作已入队且尚未开始执行时置位。
 * 用于把重复动作合并成一次，避免 RC 通道抖动时把队列灌满（见 post()）。 */
static volatile uint32_t s_pending_mask = 0;
static portMUX_TYPE s_pending_lock = portMUX_INITIALIZER_UNLOCKED;

/* 协议切换的最新目标：多次切换只保留最后一次，避免合并时丢掉最终意图。 */
static backend_id_t s_switch_target = BACKEND_DJI;

/**
 * @brief 该动作是否可合并
 *
 * 只合并「状态型/幂等」动作：重复请求同一状态没有额外语义，合并掉是安全的，
 * 而这正是 RC 通道抖动时灌满队列的来源（RECORD 随通道反复进出区间）。
 * 边沿型动作（单击切换、快门）绝不能合并 —— 合并会吃掉用户的连续输入。
 */
static bool action_coalescable(ctrl_action_t action)
{
    switch (action) {
    case CTRL_RECORD_START:
    case CTRL_RECORD_STOP:
    case CTRL_SWITCH_PROTOCOL:
    case CTRL_PAIRING_ENTER:
    case CTRL_PAIRING_EXIT:
        return true;
    default:
        return false;
    }
}

/**
 * @brief 投递一个控制动作（非阻塞，状态型动作可合并）
 *
 * 关键约束：**绝不阻塞调用者**。调用者可能是按键扫描任务或 OSD 任务，
 * 一旦在这里阻塞，按键会彻底失灵、OSD/相机状态刷新会停摆。
 * 因此：
 *   - 队列满 → 告警并丢弃，不等待（丢一个重复动作远好过死锁）；
 *   - 可合并的同类动作已在队列中 → 跳过，从源头消除洪泛。
 */
static void post(ctrl_action_t action, backend_id_t id)
{
    if (s_ctrl_queue == NULL) {
        ESP_LOGW(TAG, "control queue not ready, drop action %d", (int)action);
        return;
    }

    if (action_coalescable(action)) {
        portENTER_CRITICAL(&s_pending_lock);
        if (action == CTRL_SWITCH_PROTOCOL) {
            s_switch_target = id; /* 多次切换只保留最后一次目标 */
        }
        bool already_pending = (s_pending_mask & (1u << action)) != 0;
        s_pending_mask |= (1u << action);
        portEXIT_CRITICAL(&s_pending_lock);

        if (already_pending) {
            return; /* 同类动作已在队列中，参数已更新，无需重复入队 */
        }
    }

    ctrl_msg_t msg = { .action = action, .id = id };
    if (xQueueSend(s_ctrl_queue, &msg, 0) != pdTRUE) {
        /* 队列满：撤销待处理标记并丢弃，绝不阻塞调用者。 */
        portENTER_CRITICAL(&s_pending_lock);
        s_pending_mask &= ~(1u << action);
        portEXIT_CRITICAL(&s_pending_lock);
        ESP_LOGW(TAG, "control queue full, drop action %d", (int)action);
    }
}

static void control_task(void *arg)
{
    (void)arg;
    ctrl_msg_t msg;

    for (;;) {
        if (xQueueReceive(s_ctrl_queue, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* 先清除待处理标记：执行期间新到的同类动作仍可再次入队（不会被丢失）。 */
        portENTER_CRITICAL(&s_pending_lock);
        s_pending_mask &= ~(1u << msg.action);
        backend_id_t target = (msg.action == CTRL_SWITCH_PROTOCOL) ? s_switch_target : msg.id;
        portEXIT_CRITICAL(&s_pending_lock);

        const camera_backend_t *be = camera_backend_active();
        switch (msg.action) {
        case CTRL_SINGLE_PRESS:
            if (be != NULL && be->single_press != NULL) be->single_press();
            break;
        case CTRL_SHUTTER:
            if (be != NULL && be->shutter != NULL) be->shutter();
            break;
        case CTRL_RECORD_START:
            if (be != NULL && be->record_start != NULL) be->record_start();
            break;
        case CTRL_RECORD_STOP:
            if (be != NULL && be->record_stop != NULL) be->record_stop();
            break;
        case CTRL_PRESET_NEXT:
            if (be != NULL && be->preset_next != NULL) be->preset_next();
            break;
        case CTRL_SLEEP_WAKE:
            if (be != NULL && be->sleep_wake != NULL) be->sleep_wake();
            break;
        case CTRL_POWER_OFF:
            if (be != NULL && be->power_off != NULL) be->power_off();
            break;
        case CTRL_WAKE_BEACON:
            if (be != NULL && be->wake_beacon != NULL) be->wake_beacon();
            break;
        case CTRL_SWITCH_PROTOCOL:
            camera_backend_switch_to(target);
            break;
        case CTRL_PAIRING_ENTER:
            pairing_enter();
            break;
        case CTRL_PAIRING_EXIT:
            pairing_exit();
            break;
        default:
            break;
        }
    }
}

int controller_init(void)
{
    if (s_ctrl_queue != NULL) {
        return 0;
    }
    s_ctrl_queue = xQueueCreate(8, sizeof(ctrl_msg_t));
    if (s_ctrl_queue == NULL) {
        ESP_LOGE(TAG, "failed to create control queue");
        return -1;
    }
    if (xTaskCreate(control_task, "controller", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create control task");
        vQueueDelete(s_ctrl_queue);
        s_ctrl_queue = NULL;
        return -1;
    }
    return 0;
}

void controller_single_press(void)   { post(CTRL_SINGLE_PRESS, BACKEND_DJI); }
void controller_shutter(void)        { post(CTRL_SHUTTER, BACKEND_DJI); }
void controller_record_start(void)   { post(CTRL_RECORD_START, BACKEND_DJI); }
void controller_record_stop(void)    { post(CTRL_RECORD_STOP, BACKEND_DJI); }
void controller_preset_next(void)    { post(CTRL_PRESET_NEXT, BACKEND_DJI); }
void controller_sleep_wake(void)     { post(CTRL_SLEEP_WAKE, BACKEND_DJI); }
void controller_power_off(void)      { post(CTRL_POWER_OFF, BACKEND_DJI); }
void controller_wake_beacon(void)    { post(CTRL_WAKE_BEACON, BACKEND_DJI); }
void controller_switch_protocol(backend_id_t id) { post(CTRL_SWITCH_PROTOCOL, id); }
void controller_pairing_enter(void)  { post(CTRL_PAIRING_ENTER, BACKEND_DJI); }
void controller_pairing_exit(void)   { post(CTRL_PAIRING_EXIT, BACKEND_DJI); }
