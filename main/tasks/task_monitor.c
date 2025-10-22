/**
 * @file task_monitor.cpp
 * @author Sid Price (sid@sidprice.com)
 * @brief
 * @version 0.1
 * @date 2025-04-20
 *
 * @copyright Copyright Sid Price (c) 2025
 *
 */

#include "esp_log.h"
#include "serial_control.h"
#include "task_monitor.h"

/**
 * @brief The queue handle for the monitor output task  
 * 
 */
QueueHandle_t task_monitor_queue;

/**
 * @brief Task to output system monitor messages
 *
 * @param pvParameters Unused parameter
 */
void task_monitor(void *pvParameters)
{
	(void)pvParameters;
	//
	// Create the input queue for the task
	//
	task_monitor_queue = xQueueCreate(MONITOR_OUTPUT_QUEUE_DEPTH, sizeof(monitor_output_message_t));
	while (1) {
		monitor_output_message_t message;
		if (xQueueReceive(task_monitor_queue, &message, portMAX_DELAY) == pdTRUE) {
			ESP_LOGI(message.tag, "%s", message.message);
		}
	}
}