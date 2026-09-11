/* SPDX-License-Identifier: MIT */

#include "web_server.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "nvs_util.h"
#include "esp_system.h"
#include "cJSON.h"

#include "camera_state.h"
#include "osd_config.h"
#include "camera_backend.h"
#include "controller.h"
#include "pairing.h"
#include "channel_map.h"
#include "light.h"
#include "fc_msp.h"
#include "gps_fusion.h"

static const char *TAG = "WEB";

#define AP_SSID "CamLink"
#define AP_IP   "192.168.4.1"

#define WIFI_NVS_NAMESPACE        "wifi_cfg"
#define WIFI_NVS_KEY_AUTO_DELAY   "auto_delay"
#define WIFI_AUTO_DELAY_DEFAULT   60

/* ---- 内嵌前端（EMBED_FILES web/index.html 生成，符号用 basename） ---- */
extern const unsigned char index_html_start[] asm("_binary_index_html_start");
extern const unsigned char index_html_end[] asm("_binary_index_html_end");

static bool s_wifi_on = false;
static uint32_t s_auto_delay = WIFI_AUTO_DELAY_DEFAULT;

/* ---- JSON 辅助 ---- */
static esp_err_t send_json(httpd_req_t *req, cJSON *obj) {
    char *s = cJSON_PrintUnformatted(obj);
    cJSON_Delete(obj);
    if (!s) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_sendstr(req, s);
    free(s);
    return err;
}

static cJSON *read_json_body(httpd_req_t *req) {
    int total = req->content_len;
    if (total <= 0 || total > 8192) return NULL;
    char *buf = malloc(total + 1);
    if (!buf) return NULL;
    int recv = 0;
    while (recv < total) {
        int r = httpd_req_recv(req, buf + recv, total - recv);
        if (r <= 0) { free(buf); return NULL; }
        recv += r;
    }
    buf[total] = '\0';
    cJSON *obj = cJSON_Parse(buf);
    free(buf);
    return obj;
}

/* ---- 处理器 ---- */

/* GET / */
static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, (ssize_t)(index_html_end - index_html_start));
    return ESP_OK;
}

/* GET /api/status */
static esp_err_t status_handler(httpd_req_t *req) {
    /* 不在此调用 camera_state_refresh()：缓存由 osd_task 单写者周期刷新，
     * 这里只读最新快照，避免从 httpd 任务竞争写缓存。 */
    const camera_state_t *st = camera_state_get();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "protocol", camera_backend_active_id());
    cJSON_AddBoolToObject(root, "paired", camera_backend_is_paired());
    cJSON_AddBoolToObject(root, "pairing", pairing_is_active());
    cJSON_AddBoolToObject(root, "wifi_on", s_wifi_on);
    cJSON_AddNumberToObject(root, "wifi_auto_delay", s_auto_delay);
    /* 飞控 GPS 的方向档位（0=读飞控 / 1=向飞控输出） */
    cJSON_AddNumberToObject(root, "gps_dir", (int)fc_msp_get_gps_dir());

    cJSON *gps = cJSON_CreateObject();
    cJSON_AddBoolToObject(gps, "connected", st->gps_connected);
    cJSON_AddBoolToObject(gps, "valid", st->gps_valid);
    cJSON_AddNumberToObject(gps, "source", st->gps_source);
    cJSON_AddNumberToObject(gps, "satellites", st->satellites);
    cJSON_AddNumberToObject(gps, "speed_ms", st->speed_ms);
    cJSON_AddNumberToObject(gps, "altitude_m", st->altitude_m);
    cJSON_AddNumberToObject(gps, "lat", st->lat);
    cJSON_AddNumberToObject(gps, "lon", st->lon);
    /* 融合的精度信息：便于判断"现在用的是哪个来源、准不准" */
    gps_fused_t fused;
    gps_fusion_get(&fused);
    cJSON_AddNumberToObject(gps, "h_acc_m", fused.h_acc_m);
    cJSON_AddNumberToObject(gps, "pdop", fused.pdop);

    /* 两个来源各自的状态：状态灯要区分「没接」和「接了但没定位」 */
    gps_sources_t srcs;
    gps_fusion_get_sources(&srcs);
    cJSON_AddBoolToObject(gps, "local_present", srcs.local_present);
    cJSON_AddBoolToObject(gps, "local_valid", srcs.local_valid);
    cJSON_AddBoolToObject(gps, "fc_present", srcs.fc_present);
    cJSON_AddBoolToObject(gps, "fc_valid", srcs.fc_valid);
    cJSON_AddItemToObject(root, "gps", gps);

    cJSON *chans = cJSON_CreateArray();
    uint16_t ch[16];
    channel_map_get_last_channels(ch);
    for (int i = 0; i < 16; i++) cJSON_AddItemToArray(chans, cJSON_CreateNumber(ch[i]));
    cJSON_AddItemToObject(root, "channels", chans);

    return send_json(req, root);
}

/* GET/PUT /api/osd */
static esp_err_t osd_get_handler(httpd_req_t *req) {
    osd_item_t slots[OSD_SLOT_COUNT];
    osd_config_get_slots(slots);
    cJSON *root = cJSON_CreateObject();
    cJSON *arr = cJSON_CreateArray();
    for (int i = 0; i < OSD_SLOT_COUNT; i++) cJSON_AddItemToArray(arr, cJSON_CreateNumber(slots[i]));
    cJSON_AddItemToObject(root, "slots", arr);
    cJSON_AddNumberToObject(root, "batt_alarm", osd_config_get_batt_alarm());
    return send_json(req, root);
}

static esp_err_t osd_put_handler(httpd_req_t *req) {
    cJSON *body = read_json_body(req);
    if (!body) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json"); return ESP_FAIL; }

    cJSON *slots = cJSON_GetObjectItem(body, "slots");
    if (cJSON_IsArray(slots) && cJSON_GetArraySize(slots) == OSD_SLOT_COUNT) {
        osd_item_t s[OSD_SLOT_COUNT];
        for (int i = 0; i < OSD_SLOT_COUNT; i++) {
            cJSON *it = cJSON_GetArrayItem(slots, i);
            int v = cJSON_IsNumber(it) ? it->valueint : 0;
            s[i] = (v >= 0 && v < OSD_ITEM_COUNT) ? (osd_item_t)v : OSD_ITEM_REC;
        }
        osd_config_set_slots(s);
    }

    cJSON *batt = cJSON_GetObjectItem(body, "batt_alarm");
    if (cJSON_IsNumber(batt)) {
        int v = batt->valueint;
        if (v < 0) v = 0;
        if (v > 100) v = 100;
        osd_config_set_batt_alarm((uint8_t)v);
    }

    cJSON_Delete(body);
    cJSON *ok = cJSON_CreateObject();
    cJSON_AddBoolToObject(ok, "ok", true);
    return send_json(req, ok);
}

/* GET/PUT /api/protocol */
static esp_err_t protocol_get_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "mode", camera_backend_active_id());
    cJSON_AddBoolToObject(root, "paired", camera_backend_is_paired());
    return send_json(req, root);
}

static esp_err_t protocol_put_handler(httpd_req_t *req) {
    cJSON *body = read_json_body(req);
    if (!body) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json"); return ESP_FAIL; }
    cJSON *mode = cJSON_GetObjectItem(body, "mode");
    if (cJSON_IsNumber(mode)) {
        backend_id_t m = (mode->valueint == 1) ? BACKEND_INSTA360 : BACKEND_DJI;
        controller_switch_protocol(m);
        if (m == BACKEND_INSTA360) {
            light_logic_flash(0, 13, 0, 2);    /* 绿灯双闪 = insta360 */
        } else {
            light_logic_flash(13, 0, 0, 2);    /* 红灯双闪 = DJI */
        }
    }
    cJSON_Delete(body);
    cJSON *ok = cJSON_CreateObject();
    cJSON_AddBoolToObject(ok, "ok", true);
    return send_json(req, ok);
}

/* POST /api/pairing/start | stop */
static esp_err_t pairing_start_handler(httpd_req_t *req) {
    controller_pairing_enter();
    cJSON *ok = cJSON_CreateObject();
    cJSON_AddBoolToObject(ok, "ok", true);
    return send_json(req, ok);
}

static esp_err_t pairing_stop_handler(httpd_req_t *req) {
    controller_pairing_exit();
    cJSON *ok = cJSON_CreateObject();
    cJSON_AddBoolToObject(ok, "ok", true);
    return send_json(req, ok);
}

/* GET/PUT /api/chmap */
static esp_err_t chmap_get_handler(httpd_req_t *req) {
    channel_binding_t b[CHAN_FUNC_COUNT];
    channel_map_get_bindings(b);
    cJSON *root = cJSON_CreateObject();
    cJSON *arr = cJSON_CreateArray();
    for (int f = 1; f < CHAN_FUNC_COUNT; f++) {
        cJSON *it = cJSON_CreateObject();
        cJSON_AddNumberToObject(it, "func", f);
        cJSON_AddNumberToObject(it, "channel", b[f].channel);
        cJSON_AddNumberToObject(it, "min", b[f].min);
        cJSON_AddNumberToObject(it, "max", b[f].max);
        cJSON_AddItemToArray(arr, it);
    }
    cJSON_AddItemToObject(root, "bindings", arr);
    return send_json(req, root);
}

static esp_err_t chmap_put_handler(httpd_req_t *req) {
    cJSON *body = read_json_body(req);
    if (!body) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json"); return ESP_FAIL; }

    channel_binding_t b[CHAN_FUNC_COUNT];
    channel_map_get_bindings(b);   /* 保留当前，逐个覆盖 */

    cJSON *bindings = cJSON_GetObjectItem(body, "bindings");
    if (cJSON_IsArray(bindings)) {
        int n = cJSON_GetArraySize(bindings);
        for (int i = 0; i < n; i++) {
            cJSON *it = cJSON_GetArrayItem(bindings, i);
            cJSON *func = cJSON_GetObjectItem(it, "func");
            cJSON *chan = cJSON_GetObjectItem(it, "channel");
            cJSON *mn   = cJSON_GetObjectItem(it, "min");
            cJSON *mx   = cJSON_GetObjectItem(it, "max");
            int f = cJSON_IsNumber(func) ? func->valueint : -1;
            if (f > 0 && f < CHAN_FUNC_COUNT) {
                b[f].channel = cJSON_IsNumber(chan) ? (uint8_t)chan->valueint : 0;
                b[f].min = cJSON_IsNumber(mn) ? (uint16_t)mn->valueint : 0;
                b[f].max = cJSON_IsNumber(mx) ? (uint16_t)mx->valueint : 0;
            }
        }
    }
    channel_map_set_bindings(b);

    cJSON_Delete(body);
    cJSON *ok = cJSON_CreateObject();
    cJSON_AddBoolToObject(ok, "ok", true);
    return send_json(req, ok);
}

/* POST /api/ota（原始二进制固件） */
static esp_err_t ota_handler(httpd_req_t *req) {
    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no update partition");
        return ESP_FAIL;
    }

    esp_ota_handle_t handle;
    esp_err_t err = esp_ota_begin(update, OTA_SIZE_UNKNOWN, &handle);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota begin failed");
        return ESP_FAIL;
    }

    char buf[1024];
    int remaining = req->content_len;
    while (remaining > 0) {
        int chunk = remaining < (int)sizeof(buf) ? remaining : (int)sizeof(buf);
        int r = httpd_req_recv(req, buf, chunk);
        if (r <= 0) {
            esp_ota_abort(handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
            return ESP_FAIL;
        }
        err = esp_ota_write(handle, buf, r);
        if (err != ESP_OK) {
            esp_ota_abort(handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota write failed");
            return ESP_FAIL;
        }
        remaining -= r;
    }

    err = esp_ota_end(handle);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota end failed");
        return ESP_FAIL;
    }
    err = esp_ota_set_boot_partition(update);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "set boot partition failed");
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "OTA OK, rebooting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

/* GET /api/config/backup */
static esp_err_t backup_handler(httpd_req_t *req) {
    osd_item_t slots[OSD_SLOT_COUNT];
    osd_config_get_slots(slots);
    channel_binding_t b[CHAN_FUNC_COUNT];
    channel_map_get_bindings(b);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "schema_version", 1);

    cJSON *osd = cJSON_CreateObject();
    cJSON *arr = cJSON_CreateArray();
    for (int i = 0; i < OSD_SLOT_COUNT; i++) cJSON_AddItemToArray(arr, cJSON_CreateNumber(slots[i]));
    cJSON_AddItemToObject(osd, "slots", arr);
    cJSON_AddNumberToObject(osd, "batt_alarm", osd_config_get_batt_alarm());
    cJSON_AddItemToObject(root, "osd", osd);

    cJSON *proto = cJSON_CreateObject();
    cJSON_AddNumberToObject(proto, "mode", camera_backend_active_id());
    cJSON_AddItemToObject(root, "protocol", proto);

    cJSON *chmap = cJSON_CreateObject();
    cJSON *barr = cJSON_CreateArray();
    for (int f = 1; f < CHAN_FUNC_COUNT; f++) {
        cJSON *it = cJSON_CreateObject();
        cJSON_AddNumberToObject(it, "func", f);
        cJSON_AddNumberToObject(it, "channel", b[f].channel);
        cJSON_AddNumberToObject(it, "min", b[f].min);
        cJSON_AddNumberToObject(it, "max", b[f].max);
        cJSON_AddItemToArray(barr, it);
    }
    cJSON_AddItemToObject(chmap, "bindings", barr);
    cJSON_AddItemToObject(root, "channel_map", chmap);

    return send_json(req, root);
}

/* POST /api/config/restore */
static esp_err_t restore_handler(httpd_req_t *req) {
    cJSON *body = read_json_body(req);
    if (!body) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json"); return ESP_FAIL; }

    cJSON *ver = cJSON_GetObjectItem(body, "schema_version");
    if (!cJSON_IsNumber(ver) || ver->valueint != 1) {
        cJSON_Delete(body);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unsupported schema_version");
        return ESP_FAIL;
    }

    cJSON *osd = cJSON_GetObjectItem(body, "osd");
    if (cJSON_IsObject(osd)) {
        cJSON *slots = cJSON_GetObjectItem(osd, "slots");
        if (cJSON_IsArray(slots) && cJSON_GetArraySize(slots) == OSD_SLOT_COUNT) {
            osd_item_t s[OSD_SLOT_COUNT];
            for (int i = 0; i < OSD_SLOT_COUNT; i++) {
                cJSON *it = cJSON_GetArrayItem(slots, i);
                int v = cJSON_IsNumber(it) ? it->valueint : 0;
                s[i] = (v >= 0 && v < OSD_ITEM_COUNT) ? (osd_item_t)v : OSD_ITEM_REC;
            }
            osd_config_set_slots(s);
        }
        cJSON *batt = cJSON_GetObjectItem(osd, "batt_alarm");
        if (cJSON_IsNumber(batt)) {
            int v = batt->valueint;
            if (v < 0) v = 0;
            if (v > 100) v = 100;
            osd_config_set_batt_alarm((uint8_t)v);
        }
    }

    cJSON *proto = cJSON_GetObjectItem(body, "protocol");
    if (cJSON_IsObject(proto)) {
        cJSON *mode = cJSON_GetObjectItem(proto, "mode");
        if (cJSON_IsNumber(mode)) {
            controller_switch_protocol((mode->valueint == 1) ? BACKEND_INSTA360 : BACKEND_DJI);
        }
    }

    cJSON *chmap = cJSON_GetObjectItem(body, "channel_map");
    if (cJSON_IsObject(chmap)) {
        cJSON *bindings = cJSON_GetObjectItem(chmap, "bindings");
        if (cJSON_IsArray(bindings)) {
            channel_binding_t b[CHAN_FUNC_COUNT];
            channel_map_get_bindings(b);
            int n = cJSON_GetArraySize(bindings);
            for (int i = 0; i < n; i++) {
                cJSON *it = cJSON_GetArrayItem(bindings, i);
                cJSON *func = cJSON_GetObjectItem(it, "func");
                cJSON *chan = cJSON_GetObjectItem(it, "channel");
                cJSON *mn = cJSON_GetObjectItem(it, "min");
                cJSON *mx = cJSON_GetObjectItem(it, "max");
                int f = cJSON_IsNumber(func) ? func->valueint : -1;
                if (f > 0 && f < CHAN_FUNC_COUNT) {
                    b[f].channel = cJSON_IsNumber(chan) ? (uint8_t)chan->valueint : 0;
                    b[f].min = cJSON_IsNumber(mn) ? (uint16_t)mn->valueint : 0;
                    b[f].max = cJSON_IsNumber(mx) ? (uint16_t)mx->valueint : 0;
                }
            }
            channel_map_set_bindings(b);
        }
    }

    cJSON_Delete(body);
    cJSON *ok = cJSON_CreateObject();
    cJSON_AddBoolToObject(ok, "ok", true);
    return send_json(req, ok);
}

/* GET/PUT /api/wifi */
static esp_err_t wifi_get_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "wifi_on", s_wifi_on);
    cJSON_AddNumberToObject(root, "auto_delay", s_auto_delay);
    return send_json(req, root);
}

static esp_err_t wifi_put_handler(httpd_req_t *req) {
    cJSON *body = read_json_body(req);
    if (!body) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json"); return ESP_FAIL; }

    cJSON *delay = cJSON_GetObjectItem(body, "auto_delay");
    if (cJSON_IsNumber(delay)) {
        web_server_wifi_auto_delay_set((uint32_t)delay->valueint);
    }

    cJSON *on = cJSON_GetObjectItem(body, "wifi_on");
    if (cJSON_IsBool(on)) {
        if (cJSON_IsTrue(on)) web_server_wifi_on();
        else web_server_wifi_off();
    }

    cJSON_Delete(body);
    cJSON *ok = cJSON_CreateObject();
    cJSON_AddBoolToObject(ok, "ok", true);
    return send_json(req, ok);
}

/* ---- Captive Portal：把任意域名解析到 AP IP，未匹配请求重定向到设置页 ---- */

/* 兜底 handler：302 重定向到设置页，触发系统「自动弹出登录页」 */
static esp_err_t captive_handler(httpd_req_t *req) {
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://" AP_IP "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* DNS 服务器（UDP 53）：对所有 A 记录查询返回 192.168.4.1 */
static void dns_server_task(void *arg) {
    (void)arg;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "dns: socket failed");
        return;
    }
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(53),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "dns: bind failed");
        close(sock);
        return;
    }

    uint8_t buf[512];
    for (;;) {
        struct sockaddr_in client;
        socklen_t client_len = sizeof(client);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&client, &client_len);
        if (n < 12) continue;

        /* 只应答标准 A 记录查询（问题段的 QTYPE/QCLASS 在尾部） */
        uint16_t qtype  = (uint16_t)((buf[n - 4] << 8) | buf[n - 3]);
        uint16_t qclass = (uint16_t)((buf[n - 2] << 8) | buf[n - 1]);
        if (qtype != 1 || qclass != 1) continue;

        uint8_t resp[512];
        memcpy(resp, buf, n);
        resp[2] = 0x81;   /* QR=1, RD=1 */
        resp[3] = 0x80;   /* RA=1, RCODE=0 (NOERROR) */
        resp[6] = 0;
        resp[7] = 1;      /* ANCOUNT = 1 */

        int p = n;
        resp[p++] = 0xC0; resp[p++] = 0x0C;                       /* 名字指针 -> 偏移 12 */
        resp[p++] = 0x00; resp[p++] = 0x01;                       /* TYPE = A */
        resp[p++] = 0x00; resp[p++] = 0x01;                       /* CLASS = IN */
        resp[p++] = 0x00; resp[p++] = 0x00; resp[p++] = 0x00; resp[p++] = 0x3C; /* TTL = 60 */
        resp[p++] = 0x00; resp[p++] = 0x04;                       /* RDLENGTH = 4 */
        resp[p++] = 192; resp[p++] = 168; resp[p++] = 4; resp[p++] = 1;       /* 192.168.4.1 */

        sendto(sock, resp, p, 0, (struct sockaddr *)&client, client_len);
    }
    close(sock);
}

/* PUT /api/gps-dir —— 切换飞控 GPS 的方向（二选一滑块开关）。
 * 0 = 读飞控 GPS（飞控自己接了 GPS）；1 = 向飞控输出 GPS（飞控没接，gps_provider=MSP）。
 * 两者物理互斥：都开会让我们喂进去的数据被飞控回读回来。 */
static esp_err_t gps_dir_put_handler(httpd_req_t *req) {
    cJSON *body = read_json_body(req);
    if (!body) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json"); return ESP_FAIL; }

    cJSON *dir = cJSON_GetObjectItem(body, "dir");
    if (cJSON_IsNumber(dir)) {
        fc_msp_set_gps_dir((dir->valueint == 1) ? FC_MSP_GPS_WRITE : FC_MSP_GPS_READ);
    }
    cJSON_Delete(body);

    cJSON *ok = cJSON_CreateObject();
    cJSON_AddNumberToObject(ok, "dir", (int)fc_msp_get_gps_dir());
    return send_json(req, ok);
}

/* POST /api/factory-reset —— 恢复默认设置：清空全部设置后重启。
 *
 * 直接擦掉整个 NVS 分区：6 个命名空间（协议 / 对频 / 通道映射 / insta360 /
 * OSD / WiFi）一次清干净，不必逐个删键，将来新增的设置项也不会被漏掉。 */
static esp_err_t factory_reset_handler(httpd_req_t *req) {
    ESP_LOGW(TAG, "factory reset requested: erasing NVS and rebooting");

    cJSON *ok = cJSON_CreateObject();
    cJSON_AddBoolToObject(ok, "ok", true);
    esp_err_t err = send_json(req, ok);

    /* 先把响应发出去再擦写：擦除 + 重启很快，等浏览器收到 200 才开始动手，
     * 前端就不会把「连接中断」误判成失败。 */
    vTaskDelay(pdMS_TO_TICKS(300));
    nvs_flash_erase();
    esp_restart();

    return err;   /* 走不到，仅为消除「缺返回值」告警 */
}

static void dns_server_init(void) {
    xTaskCreate(dns_server_task, "dns_server", 4096, NULL, 5, NULL);
}

/* ---- URI 注册 ---- */
static void register_handlers(httpd_handle_t server) {
    httpd_uri_t u;

#define URI(path, fn, m) do { \
        u = (httpd_uri_t){ .uri = path, .method = m, .handler = fn, .user_ctx = NULL }; \
        httpd_register_uri_handler(server, &u); \
    } while (0)

    URI("/",                     index_handler,           HTTP_GET);
    URI("/api/status",           status_handler,          HTTP_GET);
    URI("/api/osd",              osd_get_handler,         HTTP_GET);
    URI("/api/osd",              osd_put_handler,         HTTP_PUT);
    URI("/api/protocol",         protocol_get_handler,    HTTP_GET);
    URI("/api/protocol",         protocol_put_handler,    HTTP_PUT);
    URI("/api/pairing/start",    pairing_start_handler,   HTTP_POST);
    URI("/api/pairing/stop",     pairing_stop_handler,    HTTP_POST);
    URI("/api/chmap",            chmap_get_handler,       HTTP_GET);
    URI("/api/chmap",            chmap_put_handler,       HTTP_PUT);
    URI("/api/ota",              ota_handler,             HTTP_POST);
    URI("/api/config/backup",    backup_handler,          HTTP_GET);
    URI("/api/config/restore",   restore_handler,         HTTP_POST);
    URI("/api/wifi",             wifi_get_handler,        HTTP_GET);
    URI("/api/wifi",             wifi_put_handler,        HTTP_PUT);
    URI("/api/gps-dir",          gps_dir_put_handler,     HTTP_PUT);
    URI("/api/factory-reset",    factory_reset_handler,   HTTP_POST);
    URI("/*",                    captive_handler,         HTTP_GET);

#undef URI
}

/* ---- WiFi SoftAP ---- */

static void load_auto_delay(void) {
    s_auto_delay = WIFI_AUTO_DELAY_DEFAULT;
    if (nvs_ensure_ready() == ESP_OK) {
        nvs_handle_t handle;
        if (nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
            uint32_t v = 0;
            if (nvs_get_u32(handle, WIFI_NVS_KEY_AUTO_DELAY, &v) == ESP_OK && v > 0) {
                s_auto_delay = v;
            }
            nvs_close(handle);
        }
    }
    ESP_LOGI(TAG, "WiFi auto-delay = %lu s", (unsigned long)s_auto_delay);
}

uint32_t web_server_wifi_auto_delay_get(void) {
    return s_auto_delay;
}

void web_server_wifi_auto_delay_set(uint32_t sec) {
    if (sec == 0) sec = WIFI_AUTO_DELAY_DEFAULT;
    s_auto_delay = sec;
    if (nvs_ensure_ready() == ESP_OK) {
        nvs_handle_t handle;
        if (nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
            esp_err_t err = nvs_set_u32(handle, WIFI_NVS_KEY_AUTO_DELAY, sec);
            if (err == ESP_OK) err = nvs_commit(handle);
            nvs_close(handle);
        }
    }
    ESP_LOGI(TAG, "WiFi auto-delay set to %lu s", (unsigned long)sec);
}

/* 只初始化 WiFi（不启动 AP），默认 WiFi 关闭 */
static void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .channel = 1,
            .password = "",
            .max_connection = 4,
            .authmode = WIFI_AUTH_OPEN,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_LOGI(TAG, "WiFi SoftAP configured (off by default)");
}

void web_server_wifi_on(void) {
    if (s_wifi_on) return;
    /* WiFi 配置模式：停止相机连接和对频（WiFi 与 BLE 互斥），LED 切流水灯 */
    pairing_exit();
    camera_backend_stop_current();
    light_logic_set_rainbow(true);
    esp_wifi_start();
    s_wifi_on = true;
    ESP_LOGI(TAG, "WiFi AP started");
}

void web_server_wifi_off(void) {
    if (!s_wifi_on) return;
    esp_wifi_stop();
    s_wifi_on = false;
    light_logic_set_rainbow(false);
    ESP_LOGI(TAG, "WiFi AP stopped");
}

void web_server_wifi_toggle(void) {
    if (s_wifi_on) web_server_wifi_off();
    else web_server_wifi_on();
}

bool web_server_wifi_is_on(void) {
    return s_wifi_on;
}

/* 自动开 WiFi：上电后等待 auto_delay 秒，期间若相机连接成功则取消，否则自动开 WiFi。
 * 注意：任务函数绝不能 return —— FreeRTOS 会在 return 时 panic_abort("should not return")。
 * 结束必须用 vTaskDelete(NULL)。 */
static void auto_wifi_task(void *arg) {
    (void)arg;
    uint32_t delay = s_auto_delay;
    for (uint32_t i = 0; i < delay; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        /* 直接问活动后端，不依赖 camera_state 缓存（该缓存由 OSD 任务周期刷新，
         * 一旦 OSD 任务被拖住就会误判为「未连接」，进而错误地停掉相机连接）。 */
        const camera_backend_t *be = camera_backend_active();
        bool connected = (be != NULL && be->is_connected != NULL) ? be->is_connected() : false;
        if (connected) {
            ESP_LOGI(TAG, "camera connected within %lu s, skip auto-WiFi start", (unsigned long)(i + 1));
            vTaskDelete(NULL);
            return;  /* 保险：vTaskDelete 之后不会执行到这里，避免编译器告警 */
        }
    }
    web_server_wifi_on();
    ESP_LOGI(TAG, "auto-WiFi: no camera connection for %lu s, AP started", (unsigned long)delay);
    vTaskDelete(NULL);
}

/* 自动关 WiFi：WiFi 打开后若 auto_delay 秒内无 STA（手机/电脑）连接则自动关闭 */
static void wifi_idle_task(void *arg) {
    (void)arg;
    uint32_t idle = 0;
    for (;;) {
        if (!s_wifi_on) {
            idle = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        wifi_sta_list_t sta_list;
        bool has_sta = false;
        if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK && sta_list.num > 0) {
            has_sta = true;
        }

        if (has_sta) {
            idle = 0;
        } else {
            idle++;
            if (idle >= s_auto_delay) {
                web_server_wifi_off();
                ESP_LOGI(TAG, "auto-WiFi-off: no station for %lu s", (unsigned long)idle);
                idle = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int web_server_init(void) {
    /* 对频需要 BLE 独占射频，把「关闭 WiFi」注册给 pairing 编排。 */
    pairing_set_wifi_off_cb(web_server_wifi_off);

    load_auto_delay();
    wifi_init_softap();
    dns_server_init();
    xTaskCreate(auto_wifi_task, "auto_wifi", 4096, NULL, 2, NULL);
    xTaskCreate(wifi_idle_task, "wifi_idle", 3072, NULL, 2, NULL);

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 20;
    config.stack_size = 8192;
    config.server_port = 80;
    config.uri_match_fn = httpd_uri_match_wildcard;   /* 支持末尾星号通配兜底 */

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        return -1;
    }

    register_handlers(server);
    ESP_LOGI(TAG, "WebUI ready at http://%s/", AP_IP);
    return 0;
}
