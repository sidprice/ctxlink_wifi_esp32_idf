/**
 * @file task_server.cpp
 * @author Sid Price (sid@sidprice.com)
 * @brief Socket based server for ctxLink ESP32 Wi-Fi adapter
 * @version 0.1
 * @date 2025-03-24
 *
 * @copyright Copyright Sid Price (c) 2025
 *
 */

#include <lwip/sockets.h>

#include "protocol.h"
#include "serial_control.h"
#include "task_client.h"
#include "task_server.h"
#include "task_spi_comms.h"

// #include "debug.h"

#include "ctxlink.h"

#include "custom_assert.h"

#define TAG "Server Task"

QueueHandle_t server_queue;

#define server_queue_length 32

/**
 * @brief Configure the server
 *
 * @param port The port number to bind the server to
 * @param server_fd The file descriptor for the server socket
 * @param server_addr The address structure for the server
 * @return true if the server was configured successfully, false otherwise
 */
bool configure_server(in_port_t *port, int *server_fd, struct sockaddr_in *server_addr)
{
	// Create socket
	*server_fd = socket(AF_INET, SOCK_STREAM, 0);

	// Set up server address
	server_addr->sin_family = AF_INET;
	server_addr->sin_addr.s_addr = INADDR_ANY;
	server_addr->sin_port = htons(*port);

	// Bind socket to address
	if (bind(*server_fd, (struct sockaddr *)server_addr, sizeof(*server_addr)) < 0) {
		MON(TAG, "Socket bind failed -> ");
		MON_PRINTF(TAG, "%d", errno);
		return false;
	}

	// Listen for incoming connections
	if (listen(*server_fd, 5) < 0) { // TODO: Test for single client operation, set to 1 or 0?
		MON(TAG, "Socket listen failed -> ");
		MON_PRINTF(TAG, "%d", errno);
		return false;
	}
	return true;
}

/**
 * @brief Task to handle the GDB Wi-Fi server
 *
 * @param pvParameters
 */
void task_wifi_server(void *pvParameters)
{
	server_task_params_t *server_params = (server_task_params_t *)pvParameters;
	int server_fd, client_fd;
	struct sockaddr_in server_addr, client_addr;

	server_queue = xQueueCreate(server_queue_length,
		sizeof(uint8_t *));                     // Create the queue for the GDB server task
	server_params->server_queue = server_queue; // Save the queue to the server task parameters
	socklen_t addr_len = sizeof(client_addr);

	in_port_t port = (in_port_t)server_params->port; // Recover the port number for this task
	//
	if (!configure_server(&port, &server_fd, &server_addr)) {
		MON_PRINTF(TAG, "Failed to configure %s server", server_params->server_name);
	}

	MON_PRINTF(TAG, "%s server task started", server_params->server_name);
	//
	// Main server loop
	//
	while (true) {
		// Accept incoming connections
		MON_PRINTF(TAG, "%s server waiting for client on port %d.", server_params->server_name, port);
		while (true) {
			client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &addr_len);
			if (client_fd < 0) {
				MON(TAG, "Socket accept failed");
				MON_PRINTF(TAG, "%d", errno);
				return; // TODO Deal with error, cannot return!
			}
			MON(TAG, "Client connected");
			// Set the client socket to non-blocking mode
			int flags = fcntl(client_fd, F_GETFL, 0);
			flags = 1;
			setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &flags, sizeof(flags));
			server_params->client_fd = client_fd; // Save the client file descriptor for the client task
			//
			// Start a task to handle this new client
			//
			xTaskCreate(task_client, "ClientTask", 4096, (void *)server_params, 5, NULL);
			//
			//  Server data handling loop
			//
			while (true) {
				BaseType_t result;
				uint8_t *message;
				//
				// Check for packets from SPI task
				//
				result = xQueueReceive(server_queue, &message, portMAX_DELAY);
				if (result == pdTRUE) {
					size_t bytes_sent;
					size_t packet_size;
					protocol_packet_type_e packet_type;
					uint8_t *packet_data;
					CUSTOM_ASSERT(message[0] == 0xDE);
					protocol_split(message, &packet_size, &packet_type, &packet_data);

					switch (packet_type) {
					case PROTOCOL_PACKET_TYPE_TO_CLIENT: {
						MON(TAG, "Packet to client");
						while (packet_size > 0) {
							bytes_sent = send(client_fd, packet_data, packet_size, 0);
							if (bytes_sent < 0) {
								MON(TAG, "Socket send failed: ");
								MON_PRINTF(TAG, "%d", errno);
								break;
							}
							packet_size -= bytes_sent;
							packet_data += bytes_sent;
						}
						break;
					}

					case PROTOCOL_PACKET_TYPE_COMMAND: {
						protocol_packet_command_s *command_packet = (protocol_packet_command_s *)packet_data;
						if (command_packet->command == PROTOCOL_PACKET_TYPE_CMD_SHUTDOWN_GDB_SERVER) {
							MON(TAG, "Close Client/Server Sockets");
							close(client_fd);
							client_fd = -1;
							close(server_fd);
							server_fd = -1;
						} else if (command_packet->command == PROTOCOL_PACKET_TYPE_CMD_START_GDB_SERVER) {
							configure_server(&port, &server_fd, &server_addr);
							MON(TAG, "Reconfigured Server");
						} else {
							MON(TAG, "Unknown command received");
						}
						break;
					}

					default:
						MON(TAG, "Unknown packet type received");
						break;
					}
				}
			}
		}
	}
}
