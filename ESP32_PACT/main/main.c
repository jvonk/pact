/**
 * @brief ESP32 BLE iBeacon scanner and advertising (ctrl/data over MQTT)
 **/
// Copyright © 2020, Coert and Johan Vonk
// SPDX-License-Identifier: MIT
#include <sdkconfig.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_http_server.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <esp_ota_ops.h>
#include <esp_bt_device.h>

#include <ota_task.h>
#include <reset_task.h>
#include "ipc_msgs.h"
#include "mqtt_task.h"
#include "ble_task.h"

static char const * const TAG = "main_app";
static EventGroupHandle_t _wifi_event_group;
typedef enum {
    WIFI_EVENT_CONNECTED = BIT0
} my_wifi_event_t;

void _init_nvs_flash(void)
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
}

static void
_wifiStaStart(void * arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    ESP_LOGI(TAG, "STA start");
    esp_wifi_connect();
}

static void
_wifiDisconnectHandler(void * arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    ESP_LOGI(TAG, "WiFi disconnected");
    xEventGroupClearBits(_wifi_event_group, WIFI_EVENT_CONNECTED);

    // this would be a good place to stop httpd (if running)

    esp_wifi_connect();
}

static void
_wifiConnectHandler(void * arg, esp_event_base_t event_base,  int32_t event_id, void * event_data)
{
    ESP_LOGI(TAG, "Wifi connected");
    xEventGroupSetBits(_wifi_event_group, WIFI_EVENT_CONNECTED);

    // this would be a good place to start httpd (if running)

    //pool_mdns_init();
}

static void
_connect2wifi(void)
{
    _wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();  // init WiFi with configuration from non-volatile storage
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    static httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_START, &_wifiStaStart, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &_wifiDisconnectHandler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &_wifiConnectHandler, &server));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    if (strlen(CONFIG_WIFI_SSID) && strlen(CONFIG_WIFI_PASSWD)) {
        wifi_config_t wifi_config = {
            .sta = {
                .ssid = CONFIG_WIFI_SSID,
                .password = CONFIG_WIFI_PASSWD
            }
        };
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    };
    ESP_ERROR_CHECK(esp_wifi_start());

    // wait until either the connection is established
    EventBits_t bits = xEventGroupWaitBits(_wifi_event_group, WIFI_EVENT_CONNECTED, pdFALSE, pdFALSE, portMAX_DELAY);
    if (!bits) esp_restart();  // give up
}

void app_main() {

	xTaskCreate(&reset_task, "reset_task", 4096, NULL, 5, NULL);

	_init_nvs_flash();
	_connect2wifi();  // waits for WiFi connection established

    static ipc_t ipc;
    ipc.toBleQ = xQueueCreate(2, sizeof(toBleMsg_t));
    ipc.toMqttQ = xQueueCreate(2, sizeof(toMqttMsg_t));
    assert(ipc.toBleQ && ipc.toMqttQ);

    uint8_t const * const bda = esp_bt_dev_get_address();
    bda2str(bda, ipc.dev.mac);
	bda2devName(bda, ipc.dev.name, BLE_DEVNAME_LEN);

	xTaskCreate(&ota_task, "ota_task", 2 * 4096, NULL, 5, NULL);
    xTaskCreate(&ble_task, "ble_task", 2 * 4096, &ipc, 5, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 2 * 4096, &ipc, 5, NULL);
}
