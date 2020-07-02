/**
* @brief mqtt_client_task, fowards scan results to and control messages from MQTT broker
 **/
// Copyright © 2020, Johan Vonk
// SPDX-License-Identifier: MIT

#include <sdkconfig.h>
#include <stdlib.h>
#include <string.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <mqtt_client.h>
#include <esp_ota_ops.h>

#include "mqtt_client_task.h"
#include "mqtt_msg.h"

static char const * const TAG = "mqtt_client_task";
static mqtt_client_task_ipc_t const * _ipc = NULL;

static EventGroupHandle_t _mqttEventGrp = NULL;

typedef enum {
	MQTT_EVENT_CONNECTED_BIT = BIT0
} mqttEvent_t;

static struct {
    char * data;
    char * ctrl;
    char * ctrlGroup;
} _topic;

char const * _devName = NULL;
char const * _devMAC = NULL;

static esp_mqtt_client_handle_t _client;

static void _connect2broker(void);  // forward decl

static esp_err_t
_mqttEventHandler(esp_mqtt_event_handle_t event) {

	switch (event->event_id) {
        case MQTT_EVENT_DISCONNECTED:
        	_connect2broker();
            break;
        case MQTT_EVENT_CONNECTED:
            xEventGroupSetBits(_mqttEventGrp, MQTT_EVENT_CONNECTED_BIT);
            break;
        case MQTT_EVENT_DATA:
            if (event->topic && event->data_len == event->total_data_len) {  // quietly ignores chunked messaegs

                if (strncmp("restart", event->data, event->data_len) == 0) {

                    char const * const payload = "{ \"response\": \"restarting\" }";
                    esp_mqtt_client_publish(event->client, _topic.data, payload, strlen(payload), 1, 0);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    esp_restart();

                } else if (strncmp("who", event->data, event->data_len) == 0) {

                    esp_partition_t const * const running_part = esp_ota_get_running_partition();
                    esp_app_desc_t running_app_info;
                    esp_ota_get_partition_description(running_part, &running_app_info);

                    char * format = "{ \"device\": \"%s\", \"address\": \"%s\", \"version\": \"%s.%s\", \"date\": \"%s %s\" }";
                    uint const wiggleRoom = 20;
                    uint const payloadLen = strlen(format) + BLE_DEVNAME_LEN + BLE_DEVMAC_LEN + sizeof(running_app_info.project_name) + sizeof(running_app_info.version) + sizeof(running_app_info.date) + sizeof(running_app_info.time) + wiggleRoom;
                    char * const payload = malloc(payloadLen);

                    snprintf(payload, payloadLen, format, _devName, _devMAC,
                             running_app_info.project_name, running_app_info.version,
                             running_app_info.date, running_app_info.time);

                    esp_mqtt_client_publish(event->client, _topic.data, payload, strlen(payload), 1, 0);
                    free(payload);

                } else {  // forward to ble_scan_task

                    toMqttMsg_t msg = {
                        .dataType = FROM_MQTT_MSGTYPE_CTRL,
                        .data = strndup(event->data, event->data_len)
                    };
                    if (xQueueSendToBack(_ipc->fromMqttQ, &msg, 0) != pdPASS) {
                        ESP_LOGW(TAG, "fromMqttQ full");
                        free(msg.data);
                    }
                }
            }
            break;
        default:
            break;
	}
	return ESP_OK;
}

static void
_connect2broker(void) {

    xEventGroupClearBits(_mqttEventGrp, MQTT_EVENT_CONNECTED_BIT);
    {
        const esp_mqtt_client_config_t mqtt_cfg = { .event_handle = _mqttEventHandler };
        _client = esp_mqtt_client_init(&mqtt_cfg);

        esp_mqtt_client_set_uri(_client, CONFIG_BLESCAN_MQTT_URL);
        esp_mqtt_client_start(_client);
    }
	EventBits_t bits = xEventGroupWaitBits(_mqttEventGrp, MQTT_EVENT_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
	if (!bits) esp_restart();  // give up

    esp_mqtt_client_subscribe(_client, _topic.ctrl, 1);
    esp_mqtt_client_subscribe(_client, _topic.ctrlGroup, 1);
	ESP_LOGI(TAG, "Connected to MQTT Broker");
}

void
mqtt_client_task(void * ipc) {

	_ipc = ipc;

	// first message from _ipc->toMqttQ is the BLE MAC address (tx'd by ble_scan_task)

	toMqttMsg_t msg;
	if (xQueueReceive(_ipc->toMqttQ, &msg, (TickType_t)(1000L / portTICK_PERIOD_MS)) == pdPASS) {
        if (msg.dataType != TO_MQTT_MSGTYPE_DEVMAC) {
            ESP_LOGE(TAG, "unexpected dataType(%d)", msg.dataType);
            esp_restart();
        }
        _devMAC = msg.data;
		// do not free(msg.data) as we keep refering to it
    }

	// second message from _ipc->toMqttQ is the BLE Device name (tx'd by ble_scan_task)

	if (xQueueReceive(_ipc->toMqttQ, &msg, (TickType_t)(1000L / portTICK_PERIOD_MS)) == pdPASS) {

        if (msg.dataType != TO_MQTT_MSGTYPE_DEVNAME) {
            ESP_LOGE(TAG, "unexpected dataType(%d)", msg.dataType);
            esp_restart();
        }
		_devName = msg.data;
        _topic.data     = malloc(strlen(CONFIG_BLESCAN_MQTT_DATA_TOPIC) + 1 + strlen(_devName) + 1);
        _topic.ctrl      = malloc(strlen(CONFIG_BLESCAN_MQTT_CTRL_TOPIC) + 1 + strlen(_devName) + 1);
        _topic.ctrlGroup = malloc(strlen(CONFIG_BLESCAN_MQTT_CTRL_TOPIC) + 1);
		sprintf(_topic.data, "%s/%s", CONFIG_BLESCAN_MQTT_DATA_TOPIC, _devName);  // sent msgs
        sprintf(_topic.ctrl, "%s/%s", CONFIG_BLESCAN_MQTT_CTRL_TOPIC, _devName);  // received device specific ctrl msg
        sprintf(_topic.ctrlGroup, "%s", CONFIG_BLESCAN_MQTT_CTRL_TOPIC);          // received group ctrl msg

		ESP_LOGI(TAG, "toMqttQ Rx: devName \"%s\" => publish topic.ctrl = \"%s\"", _devName, _topic.ctrl);
		// do not free(msg.data) as we keep refering to it
	}

	// connect to MQTT broker, and subcribe to ctrl topic

	_mqttEventGrp = xEventGroupCreate();

	_connect2broker();

	// all remaining messages from _ipc->toMqttQ are iBeacon scan results formatted as JSON (tx'd by ble_scan_task)

	while (1) {
		if (xQueueReceive(_ipc->toMqttQ, &msg, (TickType_t)(1000L / portTICK_PERIOD_MS)) == pdPASS) {

            switch (msg.dataType) {
                case TO_MQTT_MSGTYPE_DATA:
                    //ESP_LOGI(TAG, "%s data %s \"%s\"", __func__, _topic.data, msg.data);
        			esp_mqtt_client_publish(_client, _topic.data, msg.data, strlen(msg.data), 1, 0);
                    break;
                case TO_MQTT_MSGTYPE_CTRL:
                    //ESP_LOGI(TAG, "%s ctrl %s \"%s\"", __func__, _topic.data, msg.data);
        			esp_mqtt_client_publish(_client, _topic.ctrl, msg.data, strlen(msg.data), 1, 0);
                    break;
                case TO_MQTT_MSGTYPE_DEVMAC:
                case TO_MQTT_MSGTYPE_DEVNAME:
                    ESP_LOGE(TAG, "unexpected dataType(%d)", msg.dataType);
                    break;
            }
			free(msg.data);
		}
	}
}