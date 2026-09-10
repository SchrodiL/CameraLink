/* SPDX-License-Identifier: MIT */

#include "controller.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "camera_backend.h"
#include "pairing.h"

typedef enum {
    CTRL_SINGLE_PRESS = 0,
    CTRL_SHUTTER,
    CTRL_RECORD_START,
    CTRL_RECORD_STOP,
    CTRL_SWITCH_PROTOCOL,
    CTRL_PAIRING_ENTER,
    CTRL_PAIRING_EXIT,
} ctrl_action_t;

typedef struct {
    ctrl_action_t action;
    backend_id_t id;
} ctrl_msg_t;

static QueueHandle_t s_ctrl_queue = NULL;

static void post(ctrl_action_t action, backend_id_t id)
{
    if (s_ctrl_queue == NULL) {
        return;
    }
    ctrl_msg_t msg = { .action = action, .id = id };
    xQueueSend(s_ctrl_queue, &msg, portMAX_DELAY);
}

static void control_task(void *arg)
{
    (void)arg;
    ctrl_msg_t msg;

    for (;;) {
        /* 200ms 超时：既串行处理控制动作，又驱动对频的周期边沿检测。 */
        if (xQueueReceive(s_ctrl_queue, &msg, pdMS_TO_TICKS(200)) == pdTRUE) {
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
            case CTRL_SWITCH_PROTOCOL:
                camera_backend_switch_to(msg.id);
                break;
            case CTRL_PAIRING_ENTER:
                pairing_enter();
                break;
            case CTRL_PAIRING_EXIT:
                pairing_exit();
                break;
            }
        } else {
            /* 空闲时驱动对频检测（insta360 连接边沿 + DJI 扫描结果应用）。 */
            pairing_poll();
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
        return -1;
    }
    if (xTaskCreate(control_task, "controller", 4096, NULL, 3, NULL) != pdPASS) {
        return -1;
    }
    return 0;
}

void controller_single_press(void)   { post(CTRL_SINGLE_PRESS, BACKEND_DJI); }
void controller_shutter(void)        { post(CTRL_SHUTTER, BACKEND_DJI); }
void controller_record_start(void)   { post(CTRL_RECORD_START, BACKEND_DJI); }
void controller_record_stop(void)    { post(CTRL_RECORD_STOP, BACKEND_DJI); }
void controller_switch_protocol(backend_id_t id) { post(CTRL_SWITCH_PROTOCOL, id); }
void controller_pairing_enter(void)  { post(CTRL_PAIRING_ENTER, BACKEND_DJI); }
void controller_pairing_exit(void)   { post(CTRL_PAIRING_EXIT, BACKEND_DJI); }
