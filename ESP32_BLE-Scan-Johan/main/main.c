/**
 * @brief ESP32 BLE iBeacon scanner and advertising (ctrl/data over MQTT)
 **/
// Copyright © 2020, Johan Vonk
// SPDX-License-Identifier: MIT
#include <esp_event.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <sdkconfig.h>
#include <stdio.h>
#include <sys/param.h>

#include <ota_task.h>
#include <reset_task.h>
#include <ble_scan_task.h>

#include "mqtt_client_task.h"
#include "mqtt_msg.h"

static char const *const TAG = "main_app";

static EventGroupHandle_t _connectEventGroup;
typedef enum {
    CONNECT_EVENT_GOT_IPV4 = BIT(0)
} connectEvent_t;

void _init_nvs_flash(void)
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
}

static void on_got_ip(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
	// ip_event_got_ip_t * event = (ip_event_got_ip_t *)event_data;
	// event->ip_info.ip contains IP address
	xEventGroupSetBits(_connectEventGroup, CONNECT_EVENT_GOT_IPV4);
}

static void on_wifi_disconnect(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
	ESP_LOGI(TAG, "WiFi disconnected, attempting to reconnect ..");
	ESP_ERROR_CHECK(esp_wifi_connect());
}

void _connect_to_wifi(void) {
	_connectEventGroup = xEventGroupCreate();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_ERROR_CHECK(esp_wifi_connect());

	if (!xEventGroupWaitBits(_connectEventGroup, CONNECT_EVENT_GOT_IPV4, pdFALSE, pdFALSE, portMAX_DELAY)) {
        esp_restart();
    }
}

void app_main() {
	ESP_LOGI(TAG, "starting ..");
	xTaskCreate(&reset_task, "reset_task", 4096, NULL, 5, NULL);

	_init_nvs_flash();
	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	_connect_to_wifi();  // waits for WiFi connection established

	//xTaskCreate(&ota_task, "ota_task", 2 * 4096, NULL, 5, NULL);

	QueueHandle_t toMqttQ = xQueueCreate(2, sizeof(toMqttMsg_t));
	QueueHandle_t fromMqttQ = xQueueCreate(2, sizeof(fromMqttMsg_t));

	if (toMqttQ && fromMqttQ) {
		static ble_scan_task_ipc_t ble_scan_task_ipc;
		ble_scan_task_ipc.fromMqttQ = fromMqttQ;          // rx
		ble_scan_task_ipc.toMqttQ = toMqttQ;  // tx
		xTaskCreate(&ble_scan_task, "ble_scan_task", 2 * 4096, &ble_scan_task_ipc, 5, NULL);

		static mqtt_client_task_ipc_t mqtt_client_task_ipc;
		mqtt_client_task_ipc.fromMqttQ = fromMqttQ;          // tx
		mqtt_client_task_ipc.toMqttQ = toMqttQ;  // rx
		xTaskCreate(&mqtt_client_task, "mqtt_client_task", 2 * 4096, &mqtt_client_task_ipc, 5, NULL);
	}
}
