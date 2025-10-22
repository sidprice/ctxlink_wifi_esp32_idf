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

#define nREADY     GPIO_NUM_8 // GPIO pin for ctxLink nReady input
#define nSPI_READY GPIO_NUM_7 // GPIO pin for ctxLink SPI ready input

#define SPI_SS_PIN   GPIO_NUM_34 // Your custom SS pin
#define SPI_MISO_PIN GPIO_NUM_37
#define SPI_MOSI_PIN GPIO_NUM_35
#define SPI_SCK_PIN  GPIO_NUM_36

static bool is_tx = false;

#define BUFFER_SIZE 2000 // should be multiple of 4
#define QUEUE_SIZE  1

static uint8_t *tx_saved_transaction;
static uint8_t zero_transaction_buffer[BUFFER_SIZE] = {0}; // Use your max transfer size

bool system_setup_done = false;

static int attn_state = 0;

/**
 * @brief Save the passed transaction packet pointer for later transmission
 *
 * @param transaction_buffer  Pointer to the packet to be sent
 *
 *  We are about to start a TX transaction, so save the packet pointer and
 *  assert the ATTN signal to ctxLink.
 */
void spi_save_tx_transaction_buffer(uint8_t *transaction_buffer)
{
	tx_saved_transaction = transaction_buffer;
	attn_state = 0;
	gpio_set_level(ATTN, 0);
}

/**
 * @brief Callback function on transaction completed
 *
 */
void IRAM_ATTR userTransactionCallback(spi_slave_transaction_t *trans)
{
	ESP_LOGI(TAG, "Transaction complete callback");
	gpio_set_level(nSPI_READY, 1);
	attn_state = 1;
	gpio_set_level(ATTN, 1);

	if (is_tx == false) {
		assert(trans->rx_buffer != NULL);
		FREERTOS_CHECK(xQueueSendFromISR(spi_comms_queue, trans->rx_buffer, NULL));
	} else {
		ESP_LOGI(TAG, "TX transaction complete");
	}
}

/**
 * @brief Callback function, called after transaction setup is completed
 *
 */
static void IRAM_ATTR userPostSetupCallback()
{
	ESP_LOGI(TAG, "Post setup callback");
	gpio_set_level(nSPI_READY, 0); // Tell ctxLink the transaction is ready to go.
}

/**
 * @brief Interrupt handler for the SPI CS input falling transition
 *
 *  If ATTN is asserted, set up a TX transaction using the saved txtransaction
 * packet.
 *
 *  Otherwise, set up an RX transaction
 *
 *  Do nothing if ESP32 is not ready!
 */
static void IRAM_ATTR spi_ss_activated(void *arg)
{
	ESP_LOGI(TAG, "SS activated %d", attn_state);
	control_esp32_ready(false); // De-assert ESP32 is ready
	if (system_setup_done) {
		if (attn_state == 0) { // Is this a TX transaction?
			ESP_LOGI(TAG, "Setting up TX transaction");
			// Set up a transaction to send the saved transaction buffer to ctxLink
			spi_create_pending_transaction(tx_saved_transaction, NULL,
				true); // This is a pending tx transaction
		} else {
			// Set up a transaction to receive data from ctxLink
			ESP_LOGI(TAG, "Setting up RX transaction");
			spi_create_pending_transaction(NULL, get_next_spi_buffer(),
				false); // This is a pending rx transaction
		}
	}
}

/**
 * @brief Initialize the SPI peripheral for ctxLink communication
 *
 */
void initCtxLink(void)
{
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
	attn_state = 1;
	gpio_set_level(ATTN, 1);
	//
	// Setup the SPI_SS_PIN as an input with pullup and interrupt on falling edge
	//
	memset(&gpio_configuration, 0, sizeof(gpio_configuration));
	gpio_configuration.intr_type = GPIO_INTR_NEGEDGE;
	gpio_configuration.pin_bit_mask = (1ULL << SPI_SS_PIN);
	gpio_configuration.mode = GPIO_MODE_INPUT;
	gpio_configuration.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpio_configuration.pull_up_en = GPIO_PULLUP_DISABLE;
	esp_err_t err;
	err = gpio_config(&gpio_configuration);
	ESP_LOGI(TAG, "gpio_config: %d", err);
	//
	// Setup and enable the interrupt
	//
	err = gpio_install_isr_service(0);
	ESP_LOGI(TAG, "gpio_install_isr_service: %d", err);
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
		.queue_size = 1,
		.flags = 0,
		.post_setup_cb = userPostSetupCallback,
		.post_trans_cb = userTransactionCallback,
	};
	spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
	//
	// Finally enable the SS interrupt
	//
	err = gpio_isr_handler_add(SPI_SS_PIN, spi_ss_activated, NULL);
	ESP_LOGI(TAG, "gpio_isr_handler_add: %d", err);
}

/**
 * @brief Create a SPI transaction with the ctxLink module
 *
 * The slave must always be prepared for the master to send
 * a transaction. This pending transaction is created to service
 * a transaction from the master.
 *
 * TODO: Update the following comment, as it is out of date
 * Note:  When the slave has a packet to send to the master, the pending
 *        transaction will be removed from the queue, and a new one
 *        created after the slave packet has been sent to the master.
 */
void spi_create_pending_transaction(uint8_t *dma_tx_buffer, uint8_t *dma_rx_buffer, bool isTx)
{
	is_tx = isTx; // Set the transaction type
	ESP_LOGI(TAG, "Queueing %s transaction", isTx ? "TX" : "RX");
	//
	// Set up the transaction buffers depending upon the transfer direction
	//
	// Replace NULL pointers with pointers to buffers filled with zeroes.
	//
	// Note: Only a single zero-filled buffer is provided since one of the RX/TX
	// buffers must be valid!
	//
	// This keeps the buffers valid, even when not supplied by the caller
	//
	const uint8_t *tx_buf_to_use = (dma_tx_buffer == NULL) ? zero_transaction_buffer : dma_tx_buffer;
	uint8_t *rx_buf_to_use = (dma_rx_buffer == NULL) ? zero_transaction_buffer : dma_rx_buffer;
	spi_slave_transaction_t transaction;
	memset(&transaction, 0, sizeof(transaction)); // Zero out the transaction
	transaction.length = BUFFER_SIZE * 8;         // Length in bits
	transaction.tx_buffer = tx_buf_to_use;
	transaction.rx_buffer = rx_buf_to_use;
	// ESP_LOGI(TAG, "Queueing %s transaction", isTx ? "TX" : "RX");
	ESP_ERROR_CHECK(spi_slave_queue_trans(SPI2_HOST, &transaction, portMAX_DELAY));
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
