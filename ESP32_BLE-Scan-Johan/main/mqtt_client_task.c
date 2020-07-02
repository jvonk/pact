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

#include "mqtt_client_task.h"
#include "mqtt_msg.h"

static char const * const TAG = "mqtt_client_task";
static mqtt_client_task_ipc_t const * _ipc = NULL;

static EventGroupHandle_t mqtt_event_group = NULL;
typedef enum {
	MQTT_EVENT_CONNECTED_BIT = BIT0
} mqttEvent_t;

static struct {
    char * data;
    char * ctrl;
    char * ctrlGroup;
} _topic;

char const * _devName = NULL;

static esp_err_t
_mqttEventHandler(esp_mqtt_event_handle_t event) {

	switch (event->event_id) {
        case MQTT_EVENT_CONNECTED: {
            xEventGroupSetBits(mqtt_event_group, MQTT_EVENT_CONNECTED_BIT);
            break;
        }
        case MQTT_EVENT_DATA:
            if (event->topic && event->data_len == event->total_data_len) {  // quietly ignores chunked messaegs

                if (strncmp("restart", event->data, event->data_len) == 0) {

                    char const * const payload = "{ \"response\": \"restarting\" }";
                    esp_mqtt_client_publish(event->client, _topic.data, payload, strlen(payload), 1, 0);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    esp_restart();

                } else if (strncmp("who", event->data, event->data_len) == 0) {

                    char payload[20 + strlen(_devName)];
                    sprintf(payload, "{ \"devName\": \"%s\" }", _devName);
                    esp_mqtt_client_publish(event->client, _topic.data, payload, strlen(payload), 1, 0);

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

static esp_mqtt_client_handle_t
_connect2mqtt(void) {

	mqtt_event_group = xEventGroupCreate();
    xEventGroupClearBits(mqtt_event_group, MQTT_EVENT_CONNECTED_BIT);

        const esp_mqtt_client_config_t mqtt_cfg = {
            .event_handle = _mqttEventHandler};
        esp_mqtt_client_handle_t mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

        esp_mqtt_client_set_uri(mqtt_client, CONFIG_BLESCAN_MQTT_URL);
        esp_mqtt_client_start(mqtt_client);

	EventBits_t bits = xEventGroupWaitBits(mqtt_event_group, MQTT_EVENT_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
	if (!bits) esp_restart();  // give up

    esp_mqtt_client_subscribe(mqtt_client, _topic.ctrl, 1);
    esp_mqtt_client_subscribe(mqtt_client, _topic.ctrlGroup, 1);

	ESP_LOGI(TAG, "Connected to MQTT Broker");
	return mqtt_client;
}

void
mqtt_client_task(void * ipc) {

	_ipc = ipc;

	// first message from _ipc->toMqttQ is the BLE MAC address (tx'd by ble_scan_task)

	toMqttMsg_t msg;
	if (xQueueReceive(_ipc->toMqttQ, &msg, (TickType_t)(1000L / portTICK_PERIOD_MS)) == pdPASS) {

        if (msg.dataType != TO_MQTT_MSGTYPE_DEVNAME) {
            ESP_LOGE(TAG, "unexpected dataType(%d)", msg.dataType);
            esp_restart();
        }
        ESP_LOGI(TAG, "Rx Dev Name %s", msg.data);
		_devName = msg.data;
        _topic.data = malloc(strlen(CONFIG_BLESCAN_MQTT_DATA_TOPIC) + 1 + strlen(_devName) + 1);
        _topic.ctrl = malloc(strlen(CONFIG_BLESCAN_MQTT_CTRL_TOPIC) + 1 + strlen(_devName) + 1);
        _topic.ctrlGroup = malloc(strlen(CONFIG_BLESCAN_MQTT_CTRL_TOPIC) + 1);
		sprintf(_topic.data, "%s/%s", CONFIG_BLESCAN_MQTT_DATA_TOPIC, _devName);
        sprintf(_topic.ctrl, "%s/%s", CONFIG_BLESCAN_MQTT_CTRL_TOPIC, _devName);  // device specific msgs
        sprintf(_topic.ctrlGroup, "%s", CONFIG_BLESCAN_MQTT_CTRL_TOPIC);  // group msgs

		ESP_LOGI(TAG, "toMqttQ Rx: devName \"%s\" => publish topic.ctrl = \"%s\"", _devName, _topic.ctrl);
		// do not free(msg.data) as we keep refering to it
	}

	// connect to MQTT broker, and subcribe to ctrl topic (on connect)

	esp_mqtt_client_handle_t const client = _connect2mqtt();

	// remaining messages from _ipc->toMqttQ are iBeacon scan results formatted as JSON (tx'd by ble_scan_task)

	while (1) {
		if (xQueueReceive(_ipc->toMqttQ, &msg, (TickType_t)(1000L / portTICK_PERIOD_MS)) == pdPASS) {

            switch (msg.dataType) {
                case TO_MQTT_MSGTYPE_DATA:
                    //ESP_LOGI(TAG, "%s data %s \"%s\"", __func__, _topic.data, msg.data);
        			esp_mqtt_client_publish(client, _topic.data, msg.data, strlen(msg.data), 1, 0);
                    break;
                case TO_MQTT_MSGTYPE_CTRL:
                    //ESP_LOGI(TAG, "%s ctrl %s \"%s\"", __func__, _topic.data, msg.data);
        			esp_mqtt_client_publish(client, _topic.ctrl, msg.data, strlen(msg.data), 1, 0);
                    break;
                case TO_MQTT_MSGTYPE_DEVNAME:
                    ESP_LOGE(TAG, "unexpected dataType(%d)", msg.dataType);
                    break;
            }
			free(msg.data);
		}
	}
}