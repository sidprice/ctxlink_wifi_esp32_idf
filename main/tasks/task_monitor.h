/**
 * @file task_monitor.h
 * @author Sid Price (sid@sidprice.com)
 * @brief
 * @version 0.1
 * @date 2025-04-20
 *
 * @copyright Copyright Sid Price (c) 2025
 *
 */

#ifndef TASK_MONITOR_H
#define TASK_MONITOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "serial_control.h"

/**
 * @brief Define the queue handle for the monitor output task
 * 
 */
extern QueueHandle_t task_monitor_queue;

/**
 * @brief Define the maximum length of a monitor output message
 * 
 */
#define MONITOR_OUTPUT_MAX_LENGTH 120
/**
 * @brief Define the depth of the monitor output task message queue
 * 
 */
#define MONITOR_OUTPUT_QUEUE_DEPTH 10

/**
 * @brief Define the message structure for the monitor output task
 * 
 */
typedef struct {
	char tag[32];
	char message[MONITOR_OUTPUT_MAX_LENGTH];
} monitor_output_message_t;

/**
 * @brief The task that schedules monitor output messages
 * 
 * @param pvParameters 
 */
void task_monitor(void *pvParameters);

#endif // TASK_MONITOR_H