/**
 * @file task_spi_comms.h
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

#ifndef TASK_SPI_COMMS_H
#define TASK_SPI_COMMS_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern QueueHandle_t spi_comms_queue;

//
// TODO THis setting needs optimatization
#define SPI_BUFFER_SIZE 1024

void task_spi_comms(void *pvParameters);
uint8_t *get_next_spi_buffer(void);
void initSpiCommsQueue(void);
#endif // TASK_SPI_COMMS_H