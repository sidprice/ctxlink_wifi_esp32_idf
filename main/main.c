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
#include "nvs_flash.h"
#include "ctxlink.h"
#include "tasks/task_spi_comms.h"
#include "tasks/task_wifi.h"

static const char *TAG = "main";

/**
 * @brief The task handle of the Wi-Fi task
 *
 */
TaskHandle_t wifi_task_handle = 0;

void app_main(void)
{
	// Silence common noisy components
	esp_log_level_set("wifi", ESP_LOG_NONE);
	esp_log_level_set("wifi_init", ESP_LOG_NONE);
	esp_log_level_set("phy_init", ESP_LOG_NONE);
	esp_log_level_set("system_api", ESP_LOG_NONE);
	esp_log_level_set("tcpip_adapter", ESP_LOG_NONE);

	nvs_flash_init();
	vTaskDelay(pdMS_TO_TICKS(500));
	ESP_LOGI(TAG, "CtxLink ESP32 WiFi Coprocessor, v0.1");
	initCtxLink();
	//
	// Instantiate the preferences instance
	//
	// preferences_init();
	//
	// Create the SPI communications task
	//
	xTaskCreate(task_spi_comms, "SPI Comms", 8192, NULL, 2, NULL);
	//
	// Set up Wi-Fi connection and monitor status
	//
	xTaskCreate(task_wifi, "Wi-Fi", 8192, NULL, 2, &wifi_task_handle);
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
