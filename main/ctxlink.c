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
#include <driver/gpio.h>

#include "esp_attr.h"

#include "ctxlink.h"
// #include "helper.h"
// #include "serial_control.h"
// #include "tasks/task_server.h"

#include "tasks/task_spi_comms.h"

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
	gpio_set_level(ATTN, 0);
}

/**
 * @brief Callback function on transaction completed
 *
 * @param trans Pointer to the transaction that was completed
 * @param arg   Unused user argument
 */
void IRAM_ATTR userTransactionCallback(void *arg)
{
	gpio_set_level(nSPI_READY, 1);
	gpio_set_level(ATTN, 1);

	// if (is_tx == false) {
	// 	xQueueSendFromISR(spi_comms_queue, (uint8_t *)NULL, NULL);
	// }
}

/**
 * @brief Callback function, called after transaction setup is completed
 *
 * @param trans Pointer to the transaction that was set up
 * @param arg Unused user argument
 */
static void IRAM_ATTR userPostSetupCallback(void *arg)
{
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
	control_esp32_ready(false); // De-assert ESP32 is ready
	if (system_setup_done) {
		if (gpio_get_level(ATTN) == 0) { // Is this a TX transaction?
			// Set up a transaction to send the saved transaction buffer to ctxLink
			spi_create_pending_transaction(tx_saved_transaction, NULL,
				true); // This is a pending tx transaction
		} else {
			// Set up a transaction to receive data from ctxLink
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
	gpio_set_level(ATTN, 1);
	//
	// Setup the SPI_SS_PIN as an input with pullup and interrupt on falling edge
	//
	gpio_configuration.intr_type = GPIO_INTR_NEGEDGE;
	gpio_configuration.pin_bit_mask = (1ULL << SPI_SS_PIN);
	gpio_configuration.mode = GPIO_MODE_INPUT;
	gpio_configuration.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpio_configuration.pull_up_en = GPIO_PULLUP_ENABLE;
	gpio_config(&gpio_configuration);
	//
	// Setup and enable the interrupt
	//
	gpio_install_isr_service(0);
	gpio_isr_handler_add(SPI_SS_PIN, spi_ss_activated, NULL);

	// slave.setDataMode(SPI_MODE1);
	// slave.setMaxTransferSize(BUFFER_SIZE); // default: 4092 bytes
	// slave.setQueueSize(QUEUE_SIZE);        // default: 1

	// // begin() after setting
	// slave.begin(HSPI, SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_SS_PIN);
	// slave.setUserPostSetupCbAndArg(userPostSetupCallback, NULL);
	// slave.setUserPostTransCbAndArg(userTransactionCallback, NULL);
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
	// with user-defined ISR callback that is called before/after transaction
	// start you can set these callbacks and arguments before each queue()
	// slave.setUserPostSetupCbAndArg(userPostSetupCallback, NULL);
	// slave.setUserPostTransCbAndArg(userTransactionCallback, NULL);
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
	// slave.queue(tx_buf_to_use, rx_buf_to_use, BUFFER_SIZE);
	// slave.trigger();
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
