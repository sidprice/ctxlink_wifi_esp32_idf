/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "main";

void app_main(void)
{
	ESP_LOGI(TAG, "CtxLink ESP32 WiFi Coprocessor, v0.1");
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
