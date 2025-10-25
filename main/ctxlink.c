/**
 * @file ctxlink.cpp
 * @author Sid Price (sid@sidprice.com)
 * @brief SPI communication with ctxLink module
 * @version 0.1
 * @date 2025-03-21
 *
 * @copyright Copyright Sid Price (c) 2025
 *
 * This module provides support for the SPI interface
 * between the ESP32 and ctxLink.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <esp_log.h>

#include <driver/gpio.h>
#include <driver/spi_slave.h>

#include "esp_attr.h"

#include "ctxlink.h"

#include "tasks/task_spi_comms.h"

#include "custom_assert.h"

#define TAG "CtxLink"

#include "protocol.h"

#define nREADY     GPIO_NUM_8 // GPIO pin for ctxLink nReady input
#define nSPI_READY GPIO_NUM_7 // GPIO pin for ctxLink SPI ready input

#define SPI_SS_PIN   GPIO_NUM_34 // Your custom SS pin
#define SPI_MISO_PIN GPIO_NUM_37
#define SPI_MOSI_PIN GPIO_NUM_35
#define SPI_SCK_PIN  GPIO_NUM_36

#define BUFFER_SIZE 2000 // should be multiple of 4
#define QUEUE_SIZE  1

/**
 * @brief Define the SPI peripheral to be used
 * 
 */
#define SPI_HOST SPI2_HOST

bool system_setup_done = false;
static spi_slave_transaction_t transaction_1 = {0};
static spi_slave_transaction_t transaction_2 = {0};
static bool use_trans_1 = true;

/**	
 * @brief Count of MT packets queued
 * 
 * This variable is used to limit the number of MT packets
 * queued to one at a time.
 */
static uint8_t mt_packets_queued = 0;

/**
 * @brief Reset the SPI slave queue
 * 
 * @param host 
 * @return esp_err_t 
 * 
 * N.B. This function is defined in spi_slave.c but not declared in any header file.
 */
extern esp_err_t spi_slave_queue_reset(spi_host_device_t host);

/**
 * @brief Save the passed transaction for transmission
 *
 * @param transaction_buffer  Pointer to the packet to be sent
 * @param length              Length of the packet to be sent
 *
 *  We are about to start a TX transaction:
 */
void spi_queue_transaction(uint8_t *transaction_buffer, size_t length)
{
	uint8_t *tx_buffer = get_next_spi_buffer();
	uint8_t *rx_buffer = get_next_spi_buffer();

	// Save the transaction data to the DMA buffer
	ESP_LOGI(TAG, "Create pending transaction type %02hx", *(transaction_buffer + 2));
	memcpy(tx_buffer, transaction_buffer, length);
	memset(rx_buffer, 0, BUFFER_SIZE); // Clear the RX buffer
	spi_create_pending_transaction(tx_buffer, rx_buffer);
	ESP_LOGI(TAG, "Pending transaction created");
	// Signal master there's new data
	gpio_set_level(ATTN, 0);
	gpio_set_level(ATTN, 1);
}

/**
 * @brief Callback function on transaction completed
 *
 * Send the received data to the SPI comms task.
 * 
 */
void IRAM_ATTR userTransactionCallback(spi_slave_transaction_t *trans)
{
	//
	// Negate nSPI_READY to indicate we're processing the data
	//
	gpio_set_level(nSPI_READY, 1);
	//
	// Collect the types of the Tx and Rx packets completed in
	// this transaction.
	//
	uint8_t tx_packet_type = ((uint8_t *)trans->tx_buffer)[2];
	ESP_EARLY_LOGI(TAG, "Transaction complete: tx packet type %02x", tx_packet_type);
	uint8_t rx_packet_type = ((uint8_t *)trans->rx_buffer)[2];
	ESP_EARLY_LOGI(TAG, "Transaction complete: rx packet type %02x", rx_packet_type);
	if (tx_packet_type != PROTOCOL_PACKET_TYPE_EMPTY || rx_packet_type != PROTOCOL_PACKET_TYPE_EMPTY) {
		spi_queue_mt_packet();
		//
		// Forward the rx packet to the SPI comms task
		//
		uint8_t *message_buffer = get_next_spi_buffer();
		memcpy(message_buffer, trans->rx_buffer, BUFFER_SIZE);
		assert(spi_comms_queue != NULL);
		FREERTOS_CHECK(xQueueSendFromISR(spi_comms_queue, &message_buffer, NULL));
	}
}

/**
 * @brief Callback function, called after transaction setup is completed
 *
 */
static void IRAM_ATTR userPostSetupCallback(spi_slave_transaction_t *trans)
{
	ESP_EARLY_LOGI(TAG, "Post setup: trans=%p, tx_buf=%p, tx_buf[2]=0x%02x", trans, trans->tx_buffer,
		trans->tx_buffer ? ((uint8_t *)trans->tx_buffer)[2] : 0xFF);

	gpio_set_level(nSPI_READY, 0);
}

/**
 * @brief Queue an empty packet.
 * 
 * This function sets up the SPI channel to be ready to receive
 * data from ctxLink.
 * 
 * The master should read a second packet whenever it receives an MT packet.
 * 
 * Note: Only one empty packet can be queued at a time.
 */
void spi_queue_mt_packet(void)
{
	ESP_EARLY_LOGI(TAG, "Queueing MT packet");
	uint8_t *tx_buffer = get_next_spi_buffer();
	uint8_t *rx_buffer = get_next_spi_buffer();

	memset(rx_buffer, 0, BUFFER_SIZE);                    // Clear the RX buffer
	protocol_get_mt_packet(tx_buffer);                    // Ensure the MT packet is correct
	spi_create_pending_transaction(tx_buffer, rx_buffer); // This is a pending rx transaction
}

/**
 * @brief Initialize the SPI peripheral for ctxLink communication
 *
 */
void initCtxLink(void)
{
	esp_err_t err;
	ESP_LOGI(TAG, "initCtxLink");
	initSpiCommsQueue();
	//
	// Init the SPI slave bus
	//
	spi_bus_config_t buscfg = {
		.mosi_io_num = SPI_MOSI_PIN,
		.miso_io_num = SPI_MISO_PIN,
		.sclk_io_num = SPI_SCK_PIN,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = BUFFER_SIZE,
	};
	spi_slave_interface_config_t slvcfg = {
		.mode = 1,
		.spics_io_num = SPI_SS_PIN,
		.queue_size = 2, // Additional queue entry for the empty packet feature
		.flags = 0,
		.post_setup_cb = userPostSetupCallback,
		.post_trans_cb = userTransactionCallback,
	};
	err = spi_slave_initialize(SPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
	//
	// Configure the GPIO outputs
	//
	gpio_config_t gpio_configuration;
	gpio_configuration.intr_type = GPIO_INTR_DISABLE;
	gpio_configuration.mode = GPIO_MODE_OUTPUT;
	gpio_configuration.pin_bit_mask = (1ULL << nREADY) | (1ULL << nSPI_READY) | (1ULL << ATTN);
	gpio_configuration.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpio_configuration.pull_up_en = GPIO_PULLUP_DISABLE;
	gpio_config(&gpio_configuration);

	//
	// Set the startup output levels to false (high)
	//
	gpio_set_level(nREADY, 1);
	gpio_set_level(nSPI_READY, 1);
	gpio_set_level(ATTN, 1);

	//
	// Create a SPI transaction to receive data from ctxLink
	//
	spi_queue_mt_packet();
}

/**
 * @brief Create a SPI transaction with the ctxLink module
 *
 * The slave must always be prepared for the master to send
 * a transaction. This pending transaction is created to service
 * a transaction from the master.
 *
 * Note:  When the slave has a packet to send to the master, the pending
 *        transaction will be removed from the queue, and a new one
 *        created after the slave packet has been sent to the master.
 */
void spi_create_pending_transaction(uint8_t *dma_tx_buffer, uint8_t *dma_rx_buffer)
{
	spi_slave_transaction_t *trans = use_trans_1 ? &transaction_1 : &transaction_2;
	use_trans_1 = !use_trans_1;

	memset(trans, 0, sizeof(*trans));
	trans->length = BUFFER_SIZE * 8;
	trans->tx_buffer = dma_tx_buffer;
	trans->rx_buffer = dma_rx_buffer;

	ESP_ERROR_CHECK(spi_slave_queue_trans(SPI_HOST, trans, portMAX_DELAY));
}

/**
 * @brief Indicate to ctxLink the ESP32 is ready
 *
 * Initially this is asserted once a wireless connection is made, however
 * in the future it may need to be asserted in there is no Wi-Fi connection.
 * This would enable ctxLink to configure the Wi-Fi.
 */
void control_esp32_ready(bool ready)
{
	gpio_set_level(nREADY, ready ? 0 : 1);
}
