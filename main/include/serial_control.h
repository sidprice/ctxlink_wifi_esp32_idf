/**
 * @file serial_control.h
 * @author Sid Price (sid@sidprice.com)
 * @brief Macros to enable/disable serial output
 * @version 0.1
 * @date 2024-05-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include "task_monitor.h"

/**
  * @brief Macro to control Serialx.print
  * 
  * Comment the following line to turn off all serial output
  */
#define SERIAL_ON

/**
  * @brief The following macros enable/disable serial output
  * 
  */
#ifdef SERIAL_ON
#define MONITOR(call) Serial.call
#else
#define MONITOR(call)
#endif

/**
  * @brief Experimental macro to be used for message task and queuing
  *
  */

#ifdef SERIAL_ON
#define MON(TAG, TEXT)                                             \
	do {                                                           \
		monitor_output_message_t msg;                              \
		strncpy(msg.message, TEXT, MONITOR_OUTPUT_MAX_LENGTH - 1); \
		strncpy(msg.tag, TAG, sizeof(msg.tag) - 1);                \
		xQueueSendToBack(task_monitor_queue, &msg, portMAX_DELAY); \
	} while (0)

#define MON_NL(TAG, TEXT)                                                                  \
	do {                                                                                   \
		monitor_output_message_t msg;                                                      \
		strncpy(msg.tag, TAG, sizeof(msg.tag) - 1);                                        \
		strncpy(msg.message, TEXT, MONITOR_OUTPUT_MAX_LENGTH - 1);                         \
		strncat(msg.message, "\r\n", MONITOR_OUTPUT_MAX_LENGTH - strlen(msg.message) - 1); \
		xQueueSendToBack(task_monitor_queue, &msg, portMAX_DELAY);                         \
	} while (0)

#define MON_PRINTF(TAG, FORMAT, ...)                                     \
	do {                                                                 \
		monitor_output_message_t msg;                                    \
		snprintf(msg.message, sizeof(msg.message), FORMAT, __VA_ARGS__); \
		strncpy(msg.tag, TAG, sizeof(msg.tag) - 1);                      \
		xQueueSendToBack(task_monitor_queue, &msg, portMAX_DELAY);       \
	} while (0)
#endif // SERIAL_ON