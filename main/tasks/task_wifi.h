/**
 * @file task_wifi.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-04-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef TASK_WIFI_H
#define TASK_WIFI_H

#include "task_server.h"

void task_wifi(void *pvParameters);
extern server_task_params_t gdb_server_params;
extern QueueHandle_t wifi_comms_queue;
extern TaskHandle_t wifi_task_handle;
#endif