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
#include <nvs_flash.h>
#include <sys/param.h>
#include <esp_ota_ops.h>
#include <esp_bt_device.h>
#include "esp_core_dump.h"

#include <ota_task.h>
#include <reset_task.h>
#include "ipc_msgs.h"
#include "mqtt_task.h"
#include "ble_task.h"

#define ARRAYSIZE(a) (sizeof(a) / sizeof(*(a)))
#define ALIGN( type ) __attribute__((aligned( __alignof__( type ) )))
#define PACK( type )  __attribute__((aligned( __alignof__( type ) ), packed ))
#define PACK8  __attribute__((aligned( __alignof__( uint8_t ) ), packed ))
//#define MIN(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
//#define MAX(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })

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
_wifiStaStart(void * arg_void, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWD
        }
    };
    if (*wifi_config.sta.ssid == '\0') {
        esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_config);
    }
    ESP_LOGI(TAG, "Connecting to AP \"%s\" with passwd \"%s\"",
             (char const *) wifi_config.sta.ssid, (char const *) wifi_config.sta.password);
    //ipc_t * const ipc = arg_void;
    ESP_ERROR_CHECK(esp_wifi_connect());
}

static void
_wifiDisconnectHandler(void * arg_void, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    ESP_LOGI(TAG, "WiFi disconnected");
    xEventGroupClearBits(_wifi_event_group, WIFI_EVENT_CONNECTED);

    //ipc_t * const ipc = arg_void;

    // this would be a good place to stop httpd (if running)

    vTaskDelay(10000L / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(esp_wifi_connect());
}

static void
_wifiConnectHandler(void * arg_void, esp_event_base_t event_base,  int32_t event_id, void * event_data)
{
    ESP_LOGI(TAG, "Wifi connected");
    ipc_t * const ipc = arg_void;

    xEventGroupSetBits(_wifi_event_group, WIFI_EVENT_CONNECTED);
    ipc->dev.connectCnt.wifi++;

    ip_event_got_ip_t const * const event = (ip_event_got_ip_t *) event_data;
    snprintf(ipc->dev.ipAddr, WIFI_DEVIPADDR_LEN, IPSTR, IP2STR(&event->ip_info.ip));
    ESP_LOGI(TAG, "IP addr = %s", ipc->dev.ipAddr);

    // this would be a good place to start httpd (if running)
}

static void
_connect2wifi(ipc_t * const ipc)
{
    _wifi_event_group = xEventGroupCreate();
    assert(_wifi_event_group);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t const cfg = WIFI_INIT_CONFIG_DEFAULT();  // init WiFi with configuration from non-volatile storage
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_START, &_wifiStaStart, ipc));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &_wifiDisconnectHandler, ipc));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &_wifiConnectHandler, ipc));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    if (strlen(CONFIG_WIFI_SSID)) {
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
    assert(xEventGroupWaitBits(_wifi_event_group, WIFI_EVENT_CONNECTED, pdFALSE, pdFALSE, portMAX_DELAY));
}

static void
_getCoredump(void)
{
    // BIN or ELF coredumping doesn't appear ready for primetime on SDK 4.1-beta2
    esp_core_dump_init();
#if 0
    {
        esp_partition_t const * const part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, "coredump");
        if (!part) {
            ESP_LOGE(TAG, "Coredump no part 1");
            return;
        }
        esp_partition_erase_range(part, 0, part->size);
    }
#endif

    esp_partition_t const * const part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, NULL);
    if (!part) {
        ESP_LOGE(TAG, "Coredump no part");
        return;
    }

    size_t part_addr;
    size_t part_size;
    esp_err_t err = esp_core_dump_image_get(&part_addr, &part_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Coredump no image (%s)", esp_err_to_name(err));
        ESP_LOGW(TAG, " address %x > chip->size %x, ", part_addr, esp_flash_default_chip->size);
        ESP_LOGW(TAG, " address + length %x > chip->size %x", part_addr + part_size, esp_flash_default_chip->size);
        return;
    }

    size_t const chunk_len = 256;
    size_t const str_len = chunk_len * 2 + 1;
    uint8_t * const chunk = malloc(chunk_len);
    char * const str = malloc(str_len);
    assert(chunk && str);

    for (size_t offset = 0; offset < part_size; offset += chunk_len) {

        uint const read_len = MIN(chunk_len, part_size - offset);
        //ESP_LOGI(TAG, "offset=0x%x part_size=%u, read_len=%u", offset, part_size, read_len);
        if (esp_partition_read(part, offset, chunk, read_len) != ESP_OK) {
            ESP_LOGE(TAG, "Coredump read failed");
            break;
        }
        uint len = 0;
        for (uint ii = 0; ii < read_len; ii++) {
            len += snprintf(str + len, str_len - len, "%02x", chunk[ii]);
        }
        printf("%s", str);  //  2BD: to MQTT??
    }
    free(chunk);
    free(str);
}

void app_main() {

	xTaskCreate(&reset_task, "reset_task", 4096, NULL, 5, NULL);

	_init_nvs_flash();

    static ipc_t ipc;
    ipc.toBleQ = xQueueCreate(2, sizeof(toBleMsg_t));
    ipc.toMqttQ = xQueueCreate(2, sizeof(toMqttMsg_t));
    ipc.dev.connectCnt.wifi = 0;
    ipc.dev.connectCnt.mqtt = 0;
    assert(ipc.toBleQ && ipc.toMqttQ);

	_connect2wifi(&ipc);  // waits for WiFi connection established

    //_getCoredump();
    //assert(0);

	xTaskCreate(&ota_task, "ota_task", 2 * 4096, NULL, 5, NULL);
    xTaskCreate(&ble_task, "ble_task", 2 * 4096, &ipc, 5, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 2 * 4096, &ipc, 5, NULL);
}
