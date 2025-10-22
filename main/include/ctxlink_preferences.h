/**
 * @file ctxlink_preferences.cpp
 * @author Sid Price (sid@sidprice.com)
 * @brief Preferences management for ctxLink module
 * @version 0.1
 * @date 2025-07-27
 *
 * @copyright Copyright Sid Price (c) 2025
 *
 * This module provides support for ctxLink preferences
 */

#ifndef CTXLINK_PREFERENCES_H
#define CTXLINK_PREFERENCES_H

void preferences_init(void);
size_t preferences_get_wifi_parameters(char *ssid, char *password);
void preferences_save_wifi_parameters(char *ssid, char *password);
#endif // CTXLINK_PREFERENCES_H