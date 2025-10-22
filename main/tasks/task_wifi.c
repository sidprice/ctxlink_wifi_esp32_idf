/**
 * @file task_wifi.cpp
 * @author Sid Price (sid@sidprice.com)
 * @brief
 * @version 0.1
 * @date 2025-04-20
 *
 * @copyright Copyright Sid Price (c) 2025
 *
 */

#include "serial_control.h"

#include "esp_system.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "ctxlink_preferences.h"
#include "protocol.h"

#include "task_spi_comms.h"
#include "task_wifi.h"
#include "task_server.h"

#include "ctxlink.h"

//
// Wi-Fi credentials
//
static char ssid[MAX_SSID_LENGTH] = {0};
static char password[MAX_PASS_PHRASE_LENGTH] = {0};

/**
 * @brief Define the polling period for the Wi-Fi connection state
 *
 */
const size_t wifi_state_poll_period = 10000; // Expressed in milliseconds

/**
 * @brief Define some values that will be used to indicate changes in Wi-Fi state
 * 
 */
typedef enum {
	WIFI_STATE_UNKNOWN = 0xff,
	WIFI_STATE_DISCONNECTED = 0,
	WIFI_STATE_CONNECTED = 1,
} wifi_event_state_e;

/**
 * @brief The current Wi-Fi status
 *
 */
static uint8_t wifi_status = WIFI_STATE_UNKNOWN;

/**
 * @brief Structure to hold the information about the current network connection
 *
 */
static network_connection_info_s network_info;

/**
 * @brief This is the depth of the WIFI task messaging queue
 *
 */
#define wifi_comms_queue_length 4

/**
 * @brief Instantiate the GDB server parameters
 *
 * These are passed to a server instance and also used by the SPI
 * task to endure messages are routed correctly
 *
 */
server_task_params_t gdb_server_params = {
	PROTOCOL_PACKET_STATUS_TYPE_GDB_CLIENT,
	"GDB",
	GDB_SERVER_PORT,
	0,
	NULL, // Server queue
	NULL, // Server task handle
	PROTOCOL_PACKET_TYPE_FROM_GDB,
};

/**
 * @brief Handle for the GDB Server task
 *
 */
TaskHandle_t gdb_task_handle = NULL;

/**
 * @brief The Wi-Fi task message queue
 *
 * This queue is used to send messages between the other tasks and the Wi-Fi
 * task.
 */
QueueHandle_t wifi_comms_queue;

/**
 * @brief Previous Wi-Fi status
 *
 * This is used to detect changes in the Wi-Fi status.
 */
static uint8_t previous_status = WIFI_STATE_UNKNOWN;

/**
 * @brief Local connection retry count
 * 
 */
static uint8_t local_retry_count = 0;

/**
 * @brief FreeRTOS event group used to signal connection states
 * 
 */
static EventGroupHandle_t wifi_event_group;

/**
 * @brief Define the maximum connection reties
 * 
 */
#define ESP_MAXIMUM_RETRY 5

/**
 * @brief Define this modules logging tag
 * 
 */
static const char *TAG = "wifi_task";

/**
 * @brief Define the event bits used by the module
 * 
 *  The event group allows multiple bits for each event, but we only care about
 * two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries
 */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/**
 * @brief Handle Wi-Fi events
 *
 */
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		if (local_retry_count < ESP_MAXIMUM_RETRY) {
			esp_wifi_connect();
			local_retry_count++;
			ESP_LOGI(TAG, "retry to connect to the AP");
		} else {
			xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
		}
		ESP_LOGI(TAG, "connect to the AP fail");
	} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
		ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		local_retry_count = 0;
		xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
	}
}

/**
 * @brief Send a command to the server task to shut down the server
 *
 */
void wifi_send_server_command(protocol_command_type_e command)
{
	if (gdb_task_handle != NULL && server_queue != NULL) {
		protocol_packet_command_s cmd_packet = {0};
		cmd_packet.type = PROTOCOL_PACKET_TYPE_CMD;
		cmd_packet.command = command;
		xQueueSend(server_queue, &cmd_packet, 0); // Send command to GDB server task
	}
}

void wifi_disconnect(void)
{
	if (wifi_status == WIFI_STATE_CONNECTED) {
		MON(TAG, "Disconnecting Wi-Fi");
		esp_wifi_disconnect();
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait completed disconnect
		MON(TAG, "Wi-Fi disconnected");
	}
} // deinitWiFi() end

void wifi_connect(void)
{
	//
	// Set up the Wi-Fi Station
	//
	// TODO Set the hostname to something unique, using MAC perhaps?
	//
	// WiFi.setHostname("ctxLink_adapter_1");
	// MON_PRINTF(TAG, "Hostname = %s", WiFi.getHostname());
	MON(TAG, "Connecting to WiFi.");
	wifi_event_group = xEventGroupCreate();

	ESP_ERROR_CHECK(esp_netif_init());

	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	esp_event_handler_instance_t instance_any_id;
	esp_event_handler_instance_t instance_got_ip;
	ESP_ERROR_CHECK(
		esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
	ESP_ERROR_CHECK(
		esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

	wifi_config_t wifi_config = {
		.sta =
			{
				// .ssid = EXAMPLE_ESP_WIFI_SSID,
				// .password = EXAMPLE_ESP_WIFI_PASS,
				/* Authmode threshold resets to WPA2 as default if password
               * matches WPA2 standards (password len => 8). If you want to
               * connect the device to deprecated WEP/WPA networks, Please set
               * the threshold value to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set
               * the password with length and format matching to
               * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
               */
				.threshold.authmode = WIFI_AUTH_WPA2_PSK,
				.sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
				.sae_h2e_identifier = "",
			},
	};
	strcpy((char *)wifi_config.sta.ssid, ssid);
	strcpy((char *)wifi_config.sta.password, password);
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_LOGI(TAG, "wifi_init_sta finished.");

	/* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or
   * connection failed for the maximum number of re-tries (WIFI_FAIL_BIT). The
   * bits are set by event_handler() (see above) */
	EventBits_t bits =
		xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

	/* xEventGroupWaitBits() returns the bits before the call returned, hence we
   * can test which event actually happened. */
	if (bits & WIFI_CONNECTED_BIT) {
		wifi_status = WIFI_STATE_CONNECTED;
		ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", ssid, password);
	} else if (bits & WIFI_FAIL_BIT) {
		wifi_status = WIFI_STATE_DISCONNECTED;
		ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", ssid, password);
	} else {
		ESP_LOGE(TAG, "UNEXPECTED EVENT");
	}
} // wifi_connect() end

void wifi_get_net_info(void)
{
	uint8_t *message = get_next_spi_buffer();
	MON(TAG, "Sending network info");
	memcpy(message, &network_info, sizeof(network_connection_info_s));
	package_data(message, sizeof(network_connection_info_s), PROTOCOL_PACKET_TYPE_NETWORK_INFO);
	//
	// Send to ctxLink via SPI task
	xQueueSend(spi_comms_queue, &message, 0);
}

void task_wifi(void *pvParameters)
{
	(void)pvParameters; // Unused parameter
	BaseType_t result;
	static uint8_t *message;
	//
	wifi_comms_queue = xQueueCreate(wifi_comms_queue_length,
		sizeof(uint8_t *)); // Create the queue for the SPI task

	memset(ssid, 0, MAX_SSID_LENGTH);
	memset(password, 0, MAX_PASS_PHRASE_LENGTH);
	preferences_init();
	// strcpy(ssid, "Avian Ambassadors");
	// strcpy(password, "mijo498rocks");
	size_t settings_count = preferences_get_wifi_parameters(ssid, password);
	MON_PRINTF(TAG, "SSID: %s", (char *)ssid);
	MON_PRINTF(TAG, "Passphrase: %s", (char *)password);

	wifi_connect(); // Attempt to connect to Wi-Fi
	while (1) {
		//
		// Has the wifi status changed?
		//
		if (wifi_status != previous_status) {
			MON_PRINTF(TAG, "Wi-Fi status changed = %d", wifi_status);
			previous_status = wifi_status;

			switch (wifi_status) {
			case WIFI_STATE_CONNECTED: {
				MON(TAG, "Wi-Fi Connected");
				//
				// Update the current network information structure
				//
				memset(&network_info, 0, sizeof(network_connection_info_s));
				MON_PRINTF(TAG, "Wi-Fi connected to SSID: %s", ssid);
				strncpy(network_info.network_ssid, ssid, MAX_SSID_LENGTH);
				network_info.type = PROTOCOL_PACKET_STATUS_TYPE_NETWORK_CLIENT;
				network_info.connected = 0x01; // 0x01 = connected, 0x00 = disconnected
				// network_info.ip_address[0] = (uint8_t)(WiFi.localIP()[0]);
				// network_info.ip_address[1] = (uint8_t)(WiFi.localIP()[1]);
				// network_info.ip_address[2] = (uint8_t)(WiFi.localIP()[2]);
				// network_info.ip_address[3] = (uint8_t)(WiFi.localIP()[3]);
				// network_info.mac_address[0] = (uint8_t)(WiFi.macAddress()[0]);
				// network_info.mac_address[1] = (uint8_t)(WiFi.macAddress()[1]);
				// network_info.mac_address[2] = (uint8_t)(WiFi.macAddress()[2]);
				// network_info.mac_address[3] = (uint8_t)(WiFi.macAddress()[3]);
				// network_info.mac_address[4] = (uint8_t)(WiFi.macAddress()[4]);
				// network_info.mac_address[5] = (uint8_t)(WiFi.macAddress()[5]);
				// network_info.rssi = (int8_t)(WiFi.RSSI());
				//
				uint8_t *message = get_next_spi_buffer();
				memcpy(message, &network_info, sizeof(network_connection_info_s));
				package_data(message, sizeof(network_connection_info_s), PROTOCOL_PACKET_TYPE_NETWORK_INFO);
				//
				// Start the GDB server task.
				//
				if (gdb_task_handle == NULL) {
					//
					// Start the GDB Server Task
					//
					MON(TAG, "Starting GDB Server Task");
					xTaskCreate(task_wifi_server, "GDB Server", 4096, (void *)&gdb_server_params, 1, &gdb_task_handle);
				} else {
					MON(TAG, "Restart GDB Server");
					wifi_send_server_command(PROTOCOL_PACKET_TYPE_CMD_START_GDB_SERVER);
				}
				//
				// Assert ESP32 READY to ensure ctxLink knows
				//
				// TODO Not sure this is the right place for this. What happens if Wi-Fi
				// is not connected?
				//
				control_esp32_ready(true);
				vTaskDelay(pdMS_TO_TICKS(1000));
				xQueueSend(spi_comms_queue, &message,
					0); // Send network information to SPI task
				break;
			}

			case WIFI_STATE_DISCONNECTED: {
				MON(TAG, "Wi-Fi Disconnected");
				wifi_send_server_command(PROTOCOL_PACKET_TYPE_CMD_SHUTDOWN_GDB_SERVER);
				wifi_connect(); // Attempt to reconnect to Wi-Fi
				break;
			}
			default: {
				MON_PRINTF(TAG, "Wi-Fi status changed = %d", wifi_status);
				wifi_status = WIFI_STATE_DISCONNECTED;
				break;
			}
			}

		} else {
			//
			// Wi-Fi status has not changed, so just wait for a message from the other
			// tasks or spi driver
			//
			result = xQueueReceive(wifi_comms_queue, &message, wifi_state_poll_period);
			if (result == pdTRUE) {
				size_t data_length;
				size_t packet_size;
				protocol_packet_type_e packet_type;
				uint8_t *packet_data;
				packet_size = protocol_split(message, &data_length, &packet_type, &packet_data);
				//
				// Process the received packet
				//
				network_connection_info_s *conn_info = (network_connection_info_s *)packet_data;
				MON(TAG, "Network info received");
				MON_PRINTF(TAG, "SSID: %s", conn_info->network_ssid);
				MON_PRINTF(TAG, "Passphrase: %s", conn_info->pass_phrase);
				//
				// Check if the Wi-Fi is already connected
				//
				if (wifi_status == WIFI_STATE_CONNECTED) {
					//
					// Check if the network information has changed
					//
					if (strcmp(ssid, conn_info->network_ssid) != 0 || strcmp(password, conn_info->pass_phrase) != 0) {
						MON(TAG, "Wi-Fi credentials changed, reconnecting...");
						wifi_disconnect(); // Disconnect from the current Wi-Fi connection
						memset(&network_info, 0, sizeof(network_connection_info_s));
						strncpy(ssid, conn_info->network_ssid, MAX_SSID_LENGTH);
						strncpy(password, conn_info->pass_phrase, MAX_PASS_PHRASE_LENGTH);
						wifi_connect();
					} else {
						MON(TAG, "Wi-Fi credentials unchanged");
						wifi_get_net_info();
					}
				} else {
					strncpy(ssid, conn_info->network_ssid, MAX_SSID_LENGTH);
					strncpy(password, conn_info->pass_phrase, MAX_PASS_PHRASE_LENGTH);
					wifi_connect();
				}
			}
		}
	}
}
