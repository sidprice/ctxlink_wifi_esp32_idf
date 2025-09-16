/**
 * @file task_spi_comms.cpp
 * @author Sid Price (sid@sidprice.com)
 * @brief
 * @version 0.1
 * @date 2025-03-24
 *
 * @copyright Copyright Sid Price (c) 2025
 *
 * This module handles all communications between the server tasks and the ctxLink module
 * via the SPI interface.
 */

#include "serial_control.h"
#include "task_spi_comms.h"
#include "driver/spi_slave.h"
#include "ctxlink.h"
#include "protocol.h"
// #include "tasks/task_server.h"
// #include "tasks/task_wifi.h"

// #include "debug.h"

#define TAG "SPI Comms Task"

#define SPI_BUFFER_COUNT 8

static bool tx_inflight = false;

/**
 * @brief Pool of buffers for use by the SPI interface
 *
 * Note: The buffers are aligned to 4 bytes to ensure that they are suitable for use with the SPI interface. This
 *          is noted in the ESP32 SPI documentation
 */
uint8_t spi_buffers[SPI_BUFFER_COUNT][SPI_BUFFER_SIZE] __attribute__((aligned(4)));

/**
 * @brief This is the depth of the SPI task messaging queue
 *
 */
#define spi_comms_queue_length 10

/**
 * @brief The SPI task message queue
 *
 * This queue is used to send messages between the other tasks and the SPI task.
 */
QueueHandle_t spi_comms_queue;

portMUX_TYPE my_lock = portMUX_INITIALIZER_UNLOCKED;

/**
 * @brief Get the next SPI buffer
 *
 * @return uint8_t The index of the next buffer
 *
 * This function returns the index of the next buffer to be used for SPI communications.
 * The buffers are used in a round-robin fashion.
 */
uint8_t get_next_spi_buffer_index(void)
{
	static uint8_t buffer_index = 0;
	portENTER_CRITICAL(&my_lock);
	uint8_t next_buffer = buffer_index;
	buffer_index = (buffer_index + 1) % SPI_BUFFER_COUNT;
	portEXIT_CRITICAL(&my_lock);
	return next_buffer;
}

/**
 * @brief Get the next spi buffer
 *
 * @return A pointer to the buffer
 */
uint8_t *get_next_spi_buffer(void)
{
	return spi_buffers[get_next_spi_buffer_index()];
}

/**
 * @brief Get the SPI buffer at the specified index
 *
 * @param index The index of the buffer to be returned
 * @return uint8_t* Pointer to the buffer
 */
uint8_t *get_spi_buffer(uint8_t index)
{
	return spi_buffers[index];
}

/**
 * @brief Task to handle all communications between the server tasks and the ctxLink module
 *
 * @param pvParameters Not used
 *
 * This task is responsible for handling all communications between the server tasks and the ctxLink module.
 * The Server tasks use its message queue to send data to be forwarded to the ctxLink module using the SPI
 * channel.
 *
 * This task also receives messages from the spi driver and forwards them to the appropriate server task.
 *
 * The message structure is defined in task_spi_comms.h
 */

void task_spi_comms(void *pvParameters)
{
	static uint8_t *message;
	spi_comms_queue = xQueueCreate(spi_comms_queue_length, sizeof(uint8_t *)); // Create the queue for the SPI task
	//
	// TODO is this a good place for this?
	//
	system_setup_done = true;

	while (true) {
		// Wait for a message from the other tasks or spi driver
		xQueueReceive(spi_comms_queue, &message, portMAX_DELAY);
		//
		// Process the message
		//
		size_t data_length;
		size_t packet_size;
		protocol_packet_type_e packet_type;
		uint8_t *packet_data;
		packet_size = protocol_split(message, &data_length, &packet_type, &packet_data);
		switch (packet_type) {
		case PROTOCOL_PACKET_TYPE_EMPTY: {
			//  MON_NL(TAG, "TX done?");
			break;
		}
		case PROTOCOL_PACKET_TYPE_TO_GDB: {
			//
			// Send the packet to the server task
			//
			//
			// Change the message type so that the server routes it correctly.
			//
			// The server parameters ensure the message is routed to the right
			// server task.
			//
			*(message + PACKET_HEADER_SOURCE_ID) = PROTOCOL_PACKET_TYPE_TO_CLIENT;
			//
			// TODO Need to check if there is a client attached to GDB server
			//

			// xQueueSend(
			// 	gdb_server_params.server_queue, &message, portMAX_DELAY); // Send the message to the gdb server task
			break;
		}

		case PROTOCOL_PACKET_TYPE_SET_NETWORK_INFO: {
			//
			// Send the packet to the Wi-Fi task
			//
			// xQueueSend(wifi_comms_queue, &message, portMAX_DELAY); // Send the message to the Wi-Fi task
			break;
		}
		//
		// The following cases fall-through to common code
		// to send the received message to ctxLink
		//
		case PROTOCOL_PACKET_TYPE_NETWORK_INFO:
		case PROTOCOL_PACKET_TYPE_FROM_GDB:
		case PROTOCOL_PACKET_TYPE_STATUS: {
			spi_save_tx_transaction_buffer(message); // Save the transaction buffer for SPI driver
			break;
		}
		default: {
			// MON_PRINTF(TAG, "Unknown packet type -> %d", packet_type);
			break;
		}
		}
	}
}