/**
 * @file task_server.h
 * @author Sid Price (sid@sidprice.com)
 * @brief Socket based server for ctxLink ESP32 Wi-Fi adapter
 * @version 0.1
 * @date 2025-03-24
 * 
 * @copyright Copyright Sid Price (c) 2025
 * 
 */

#ifndef TASK_SERVER_H
#define TASK_SERVER_H

#include "protocol.h"

extern QueueHandle_t server_queue;

/**
 * @brief Define the server ports
 * 
 */
#define GDB_SERVER_PORT  2159
#define UART_SERVER_PORT 2160
#define SWO_SERVER_PORT  2161

/**
 * @brief Structure for the server task configuration
 * 
 */
typedef struct {
	protocol_packet_status_type_e server_type; //  Type of the server, GDB, UART, or SWO
	char server_name[32];                      // Name of the server
	uint32_t port;                             // Port number for the server
	int client_fd;                             // File descriptor for the client socket
	QueueHandle_t server_queue;                // Handle for the server task input queue
	TaskHandle_t server_task_handle;           // Handle for the server task
	protocol_packet_type_e source_type;        // Source type of the server, GDB, UART, or SWO
} server_task_params_t;

void task_wifi_server(void *pvParameters);
#define MAGIC_HI 0xbe
#define MAGIC_LO 0xef
#endif