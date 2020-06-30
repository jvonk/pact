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

static char const * const TAG = "mqtt_client_task";
static mqtt_client_task_ipc_t const * _ipc = NULL;

static EventGroupHandle_t mqtt_event_group = NULL;
typedef enum {
	MQTT_EVENT_CONNECTED_BIT = BIT0
} mqttEvent_t;

static char _bleMAC[3 * 6];  // as string

static
esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {

	switch (event->event_id) {
        case MQTT_EVENT_CONNECTED: {
            xEventGroupSetBits(mqtt_event_group, MQTT_EVENT_CONNECTED_BIT);
            break;
        }
        case MQTT_EVENT_DATA:
            if (event->topic && event->data_len == event->total_data_len) {  // quietly ignores chunked messaegs
                char* const msg = strndup(event->data, event->data_len);

                if (xQueueSendToBack(_ipc->controlQ, &msg, 0) != pdPASS) {
                    ESP_LOGW(TAG, "controlQ full");
                    free(msg);
                }
            }
            break;
        default:
            break;
	}
	return ESP_OK;
}

static
esp_mqtt_client_handle_t _connect2mqtt(void)
{
	mqtt_event_group = xEventGroupCreate();

	const esp_mqtt_client_config_t mqtt_cfg = {
	    .event_handle = mqtt_event_handler};
	esp_mqtt_client_handle_t mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

	//esp_mqtt_client_stop(mqtt_client);
	esp_mqtt_client_set_uri(mqtt_client, CONFIG_BLESCAN_MQTT_URL);
	esp_mqtt_client_start(mqtt_client);

	EventBits_t bits = xEventGroupWaitBits(mqtt_event_group, MQTT_EVENT_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
	if (!bits) esp_restart();  // give up

    char ctrlTopic[64];
    sprintf(ctrlTopic, "%s/%s", CONFIG_BLESCAN_MQTT_CTRL_TOPIC, _bleMAC);  // device specific msgs
    ESP_LOGI(TAG, "MQTT Subscribe \"%s\"", ctrlTopic);
    esp_mqtt_client_subscribe(mqtt_client, ctrlTopic, 1);

    sprintf(ctrlTopic, "%s", CONFIG_BLESCAN_MQTT_CTRL_TOPIC);  // group msgs
    ESP_LOGI(TAG, "MQTT Subscribe \"%s\"", ctrlTopic);
    esp_mqtt_client_subscribe(mqtt_client, ctrlTopic, 1);

	ESP_LOGI(TAG, "Connected to MQTT Broker");
	return mqtt_client;
}

void
mqtt_client_task(void * ipc) {

	_ipc = ipc;

	// first message from _ipc->measurementQ is the BLE MAC address (tx'd by ble_scan_task)

	char topic[64];
	char* msg;
	if (xQueueReceive(_ipc->measurementQ, &msg, (TickType_t)(1000L / portTICK_PERIOD_MS)) == pdPASS) {
		strcpy(_bleMAC, msg);
		sprintf(topic, "%s/%s", CONFIG_BLESCAN_MQTT_TOPIC, _bleMAC);
		ESP_LOGI(TAG, "measurementQ Rx: BLE MAC \"%s\" => publish topic = \"%s\"", _bleMAC, topic);
		free(msg);
	}

	// connect to MQTT broker, and subcribe to ctrl topic (on connect)

	esp_mqtt_client_handle_t const client = _connect2mqtt();

	// remaining messages from _ipc->measurementQ are iBeacon scan results formatted as JSON (tx'd by ble_scan_task)

	while (1) {
		if (xQueueReceive(_ipc->measurementQ, &msg, (TickType_t)(1000L / portTICK_PERIOD_MS)) == pdPASS) {
			int const msg_id = esp_mqtt_client_publish(client, topic, msg, strlen(msg), 1, 0);
			ESP_LOGI(TAG, "MQTT Tx: \"%s\" = \"%s\" (%d)", topic, msg, msg_id);
			free(msg);
		}
	}
}