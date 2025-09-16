/**
 * @file custom_assert.h
 * @author your name (you@domain.com)
 * @brief Custom assertion macros for debugging
 * @version 0.1
 * @date 2025-08-27
 *
 * @copyright Copyright (c) Sid Price 2025
 *
 */

#ifndef CUSTOM_ASSERT_H
#define CUSTOM_ASSERT_H
#include "esp_log.h"

#define CUSTOM_ASSERT(expr)                                                                               \
	do {                                                                                                  \
		if (!(expr)) {                                                                                    \
			ESP_LOGI("ASSERTION", "Assertion failed: %s, file %s, line %d\n", #expr, __FILE__, __LINE__); \
			while (true) {                                                                                \
				vTaskDelay(1000);                                                                         \
			}                                                                                             \
		}                                                                                                 \
	} while (0)

#endif // CUSTOM_ASSERT_H