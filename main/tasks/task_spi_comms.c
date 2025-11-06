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

#include "task_spi_comms.h"
#include "driver/spi_slave.h"
#include "ctxlink.h"
#include "protocol.h"
#include "task_server.h"
#include "task_wifi.h"

#include "custom_assert.h"

#include "mabutrace.h"

// #include "debug.h"

#define TAG "SPI Comms Task"

//
// TODO This setting needs optimatization
//
#define SPI_BUFFER_COUNT 64

static bool tx_inflight = false;

/**
 * @brief Pool of buffers for use by the SPI interface
 *
 * Note: The buffers are aligned to 4 bytes to ensure that they are suitable for use with the SPI interface. This
 *          is noted in the ESP32 SPI documentation
 */
DMA_ATTR uint8_t spi_buffers[SPI_BUFFER_COUNT][SPI_BUFFER_SIZE] __attribute__((aligned(4)));

/**
 * @brief This is the depth of the SPI task messaging queue
 *
 */
#define spi_comms_queue_length 32

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
 * 
 * These buffers are used for all inter-task communications.
 * 
 * The network client task uses one of these buffers to send the
 * input packet to the spi comms task, which then forwards it to the SPI driver.
 * 
 * Also, the Wi-Fi task uses one of these buffers to send the network information
 * to the spi comms task, which then forwards it to ctxLink.
 * 
 * Data received from ctxLink via SPI is received into one of these buffers and
 * then forwarded to the appropriate server task via its message queue. From there
 * it is sent to the server, which forwards it to the network client.
 * 
 * The buffer is cleared ready for use.
 * 
 */
uint8_t *get_next_spi_buffer(void)
{
	uint8_t *buffer = spi_buffers[get_next_spi_buffer_index()];
	memset(buffer, 0, SPI_BUFFER_SIZE);
	return buffer;
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
 * @brief Initialize the SPI communications queue
 * 
 */
void initSpiCommsQueue(void)
{
	if (spi_comms_queue == NULL) {
		spi_comms_queue = xQueueCreate(spi_comms_queue_length,
			sizeof(uint8_t *)); // Create the queue for the SPI task
		CUSTOM_ASSERT(spi_comms_queue != NULL);
	}
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
	TRC();
	//
	// TODO is this a good place for this?
	//
	system_setup_done = true;

	// esp_log_level_set(TAG, ESP_LOG_NONE);

	while (true) {
		// Wait for a message from the other tasks or spi driver
		FREERTOS_CHECK(xQueueReceive(spi_comms_queue, &message, portMAX_DELAY));
		// CUSTOM_ASSERT(message != NULL);
		//
		// Process the message
		//
		size_t data_length;
		size_t packet_size;
		protocol_packet_type_e packet_type;
		uint8_t *packet_data;
		// Split the packet into its components
		packet_size = protocol_split(message, &data_length, &packet_type, &packet_data);
		// ESP_LOGI(TAG, "Received packet type %d, size %d", packet_type, packet_size);
		switch (packet_type) {
		case PROTOCOL_PACKET_TYPE_EMPTY: {
			// ESP_LOGI(TAG, "Packet was sent to ctxLink");
			// queue_mt_packet = true;
			break;
		}
		case PROTOCOL_PACKET_TYPE_TO_GDB: {
			// ESP_LOGI(TAG, "Packet to GDB");
			// ESP_LOGI(TAG, "Packet to server task [ %02hx %02hx %02hx %02hx ... ]", packet_data[0], packet_data[1],
			// 	packet_data[2], packet_data[3]);

			//
			// Send the packet to the server task
			//
			//
			// Change the message type so that the server routes it correctly.
			//
			// The server parameters ensure the message is routed to the right
			// server task.
			//
			// TODO It looks like the wrong message offset is being used here.
			//       Check this is correct.
			*(message + PACKET_HEADER_SOURCE_ID) = PROTOCOL_PACKET_TYPE_TO_CLIENT;
			//
			// TODO Need to check if there is a client attached to GDB server
			//

			xQueueSend(
				gdb_server_params.server_queue, &message, portMAX_DELAY); // Send the message to the gdb server task
			break;
		}

		case PROTOCOL_PACKET_TYPE_SET_NETWORK_INFO: {
			//
			// Send the packet to the Wi-Fi task
			//
			xQueueSend(wifi_comms_queue, &message, portMAX_DELAY); // Send the message to the Wi-Fi task
			break;
		}
		//
		// The following cases fall-through to common code
		// to send the received message to ctxLink
		//
		case PROTOCOL_PACKET_TYPE_FROM_GDB: {
			// ESP_LOGI(TAG, "Packet from GDB [ %02hx %02hx %02hx %02hx ... ]", packet_data[0], packet_data[1],
			// 	packet_data[2], packet_data[3]);
		}
		case PROTOCOL_PACKET_TYPE_NETWORK_INFO:
		case PROTOCOL_PACKET_TYPE_STATUS: {
			// ESP_LOGI(TAG, "Packet to ctxLink");
			protocol_split(message, &data_length, &packet_type, &packet_data);
			spi_queue_transaction(
				message, data_length + PACKET_HEADER_DATA_START); // Add the header size to the packet data length
			break;
		}
		default: {
			// ESP_LOGI(TAG, "Unknown packet type %d", packet_type);
			break;
		}
		}
	}
}