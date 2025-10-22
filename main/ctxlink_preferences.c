/**
 * @file ctxlink_preferences.c
 * @author Sid Price (sid@sidprice.com)
 * @brief Preferences management for ctxLink module
 * @version 0.1
 * @date 2025-07-27
 *
 * @copyright Copyright Sid Price (c) 2025
 *
 * This module provides support for ctxLink preferences
 */

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "protocol.h"
#include "serial_control.h"

/**
 * @brief Define the logging tag for this module
 * 
 */
#define TAG "ctxlink_preferences"
/**
 * @brief The preferences key for the Wi-Fi SSID
 * 
 */
#define wifi_ssid_key "wifi_ssid"

/**
 * @brief The preferences key for the Wi-Fi password
 * 
 */
#define wifi_password_key "wifi_password"

/**
 *  @brief define the preferences nvs handle
 * 
 */
static nvs_handle_t preferences;

/**
 * @brief Initialize the preferences instance
 * 
 */
void preferences_init(void)
{
	esp_err_t err;
	err = nvs_open("ctxlink_prefs", NVS_READWRITE, &preferences);
	if (err != ESP_OK) {
		MON_PRINTF(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
		return;
	}
	MON(TAG, "Preferences NVS handle opened");
}

/**
 * @brief Save the Wi-Fi accesspoint settings
 * 
 */
void preferences_save_wifi_parameters(char *ssid, char *password)
{
	esp_err_t err;
	err = nvs_set_str(preferences, wifi_ssid_key, ssid);
	if (err != ESP_OK) {
		MON_PRINTF(TAG, "Error (%s) saving SSID!", esp_err_to_name(err));
		return;
	}
	//
	// TODO Encrypt the pass phrase
	//
	err = nvs_set_str(preferences, wifi_password_key, password);
	if (err != ESP_OK) {
		MON_PRINTF(TAG, "Error (%s) saving password!", esp_err_to_name(err));
		return;
	}
	nvs_commit(preferences); // Ensure the values are written to flash
	MON(TAG, "Wi-Fi parameters saved to preferences");
}

/**
 * @brief Get the Wi-Fi accesspoint settings
 * 
 * @param ssid Pointer to a buffer to store the SSID
 * @param password Pointer to a buffer to store the password
 * @return size_t The total length of the SSID and password, or 0 if not found
 */
size_t preferences_get_wifi_parameters(char *ssid, char *password)
{
	size_t ssid_length, password_length;
	;
	bool save = false;
	if (!ssid || !password) {
		return 0;
	}
	esp_err_t err;
	err = nvs_get_str(preferences, wifi_ssid_key, ssid, &ssid_length);
	ssid_length = strlen(ssid);
	if (ssid_length == 0) {
		ESP_LOGI(TAG, "No SSID found in preferences, using default");
		strcpy(ssid, "ctxlink_net"); // Default SSID
		ssid_length = strlen((char *)ssid);
		save = true; // Need to save the default SSID
	}
	//
	// TODO Decrypt the pass phrase
	//
	err = nvs_get_str(preferences, wifi_password_key, password, &password_length);
	password_length = strlen(password);
	if (password_length == 0) {
		MON(TAG, "No password found in preferences, using default");
		strcpy(password, "pass_phrase"); // Default pass phrase
		password_length = strlen((char *)password);
		save = true; // Need to save the default pass phrase
	}
	if (save) {
		preferences_save_wifi_parameters(ssid, password);
	}
	return (ssid_length > 0 && password_length > 0) ? ssid_length + password_length : 0;
}
