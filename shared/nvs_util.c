/* SPDX-License-Identifier: MIT */

#include "nvs_util.h"

#include "nvs_flash.h"

esp_err_t nvs_ensure_ready(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}
