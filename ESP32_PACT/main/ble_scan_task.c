/**
 * @brief BLE iBeacon advertisement or scanning
 **/
// Copyright © 2020, Coert Vonk
// SPDX-License-Identifier: MIT

#include <sdkconfig.h>
#include <stdlib.h>
#include <string.h>
#include <esp_bt.h>
#include <esp_bt_defs.h>
#include <esp_bt_device.h>
#include <esp_bt_main.h>
#include <esp_event.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_defs.h>
#include <esp_gattc_api.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_ibeacon_api.h>

#include "ble_scan_task.h"
#include "mqtt_msg.h"

#define ARRAYSIZE(a) (sizeof(a) / sizeof(*(a)))
#define ALIGN( type ) __attribute__((aligned( __alignof__( type ) )))
#define PACK( type )  __attribute__((aligned( __alignof__( type ) ), packed ))
#define PACK8  __attribute__((aligned( __alignof__( uint8_t ) ), packed ))

static char const * const TAG = "ble_scan_task";
static ble_scan_task_ipc_t * ipc = NULL;

static EventGroupHandle_t ble_event_group = NULL;
typedef enum {
	BLE_EVENT_SCAN_PARAM_SET_COMPLETE = BIT0,
	BLE_EVENT_SCAN_START_COMPLETE = BIT1,
	BLE_EVENT_SCAN_STOP_COMPLETE = BIT2,
	BLE_EVENT_ADV_DATA_RAW_SET_COMPLETE = BIT3,
	BLE_EVENT_ADV_START_COMPLETE = BIT4,
	BLE_EVENT_ADV_STOP_COMPLETE = BIT5
} bleEvent_t;

typedef enum {  // ESP32 can only do one function at a time (SCAN || ADVERTISE)
	BLE_MODE_IDLE = 1,
	BLE_MODE_SCAN,
	BLE_MODE_ADV
} bleMode_t;

extern esp_ble_ibeacon_vendor_t vendor_config;

char *
_bla2str(uint8_t const * const bda, char * const str) {

    uint len = 0;
    for (uint ii = 0; ii < ESP_BD_ADDR_LEN; ii++) {
        len += sprintf(str + len, "%02x", bda[ii]);
        if (ii < ESP_BD_ADDR_LEN - 1) {
            str[len++] = ':';
        }
    }
    return str;
}

static void
_bleGapHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {

	esp_err_t err;

	switch (event) {
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if ((err = param->adv_start_cmpl.status) == ESP_BT_STATUS_SUCCESS) {
                xEventGroupSetBits(ble_event_group, BLE_EVENT_ADV_START_COMPLETE);
            } else {
                ESP_LOGE(TAG, "Adv start failed: %s", esp_err_to_name(err));
            }
            break;
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            xEventGroupSetBits(ble_event_group, BLE_EVENT_ADV_DATA_RAW_SET_COMPLETE);
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if ((err = param->adv_stop_cmpl.status) == ESP_BT_STATUS_SUCCESS) {
                xEventGroupSetBits(ble_event_group, BLE_EVENT_ADV_STOP_COMPLETE);
            } else {
                ESP_LOGE(TAG, "Adv stop failed: %s", esp_err_to_name(err));
            }
            break;

        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            xEventGroupSetBits(ble_event_group, BLE_EVENT_SCAN_PARAM_SET_COMPLETE);
            break;
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if ((err = param->scan_start_cmpl.status) == ESP_BT_STATUS_SUCCESS) {
                xEventGroupSetBits(ble_event_group, BLE_EVENT_SCAN_START_COMPLETE);
            } else {
                ESP_LOGE(TAG, "Scan start failed: %s", esp_err_to_name(err));
            }
            break;
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            if ((err = param->scan_stop_cmpl.status) == ESP_BT_STATUS_SUCCESS) {
                xEventGroupSetBits(ble_event_group, BLE_EVENT_SCAN_STOP_COMPLETE);
            } else {
                ESP_LOGE(TAG, "Scan stop failed: %s", esp_err_to_name(err));
            }
            break;

        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t * const scan_result = (esp_ble_gap_cb_param_t *)param;

            if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT &&
                esp_ble_is_ibeacon_packet(scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len)) {

                // format iBeacon scan result as JSON

                esp_ble_ibeacon_t const * const ibeacon_data = (esp_ble_ibeacon_t *)(scan_result->scan_rst.ble_adv);

                uint len = 0;
                char payload[256];
                len += sprintf(payload + len, "{ \"address\": \"");
                for (uint ii = 0; ii < ESP_BD_ADDR_LEN; ii++) {
                    len += sprintf(payload + len, "%02x%c", scan_result->scan_rst.bda[ii], (ii < ESP_BD_ADDR_LEN - 1) ? ':' : '"');
                }
                len += sprintf(payload + len, ", \"txPwr\": %d", ibeacon_data->ibeacon_vendor.measured_power);
                len += sprintf(payload + len, ", \"RSSI\": %d }", scan_result->scan_rst.rssi);

                //  forward to ipc->toMqttQ

                toMqttMsg_t msg = {
                    .dataType = TO_MQTT_MSGTYPE_DATA,
                    .data = strdup(payload)
                };
                if (xQueueSendToBack(ipc->toMqttQ, &msg, 0) != pdPASS) {
                    free(msg.data);
                }
            }
        }
        default:
            break;
	}
}

static void
_initIbeacon(void) {

	esp_bluedroid_init();
	esp_bluedroid_enable();

	esp_err_t status = esp_ble_gap_register_callback(_bleGapHandler);
	if (status != ESP_OK) ESP_LOGE(TAG, "gap register error: %s", esp_err_to_name(status));
}

static void
_bleStartScan(uint16_t const scan_window) {

	xEventGroupClearBits(ble_event_group, BLE_EVENT_SCAN_PARAM_SET_COMPLETE);
	{
        static esp_ble_scan_params_t ble_scan_params = {
            .scan_type = BLE_SCAN_TYPE_ACTIVE,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
            .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
        };
        ble_scan_params.scan_interval = scan_window + 0x20, // time between start of scans [n * 0.625 msec]
        ble_scan_params.scan_window = scan_window,          // scan duration               [n * 0.625 msec]

		esp_ble_gap_set_scan_params(&ble_scan_params);
	}
	xEventGroupWaitBits(ble_event_group, BLE_EVENT_SCAN_PARAM_SET_COMPLETE, pdFALSE, pdFALSE, portMAX_DELAY);

	xEventGroupClearBits(ble_event_group, BLE_EVENT_SCAN_START_COMPLETE);
    {
        uint32_t duration = 0;  // [sec], 0 means scan permanently
        esp_ble_gap_start_scanning(duration);
    }
	xEventGroupWaitBits(ble_event_group, BLE_EVENT_SCAN_START_COMPLETE, pdFALSE, pdFALSE, portMAX_DELAY);
	ESP_LOGI(TAG, "STARTED scanning");
}

static void
_bleStopScan(void) {

	xEventGroupClearBits(ble_event_group, BLE_EVENT_SCAN_STOP_COMPLETE);
    {
        if (esp_ble_gap_stop_scanning() != ESP_OK) {
            ESP_LOGI(TAG, "ERR stopping scanning");
        }
    }
	xEventGroupWaitBits(ble_event_group, BLE_EVENT_SCAN_STOP_COMPLETE, pdFALSE, pdFALSE, portMAX_DELAY);
	ESP_LOGI(TAG, "STOPPED scanning");
}

static void
_bleStartAdv(uint16_t const adv_int_max) {

	xEventGroupClearBits(ble_event_group, BLE_EVENT_ADV_DATA_RAW_SET_COMPLETE);
	{
		esp_ble_ibeacon_t ibeacon_adv_data;
		esp_err_t status = esp_ble_config_ibeacon_data(&vendor_config, &ibeacon_adv_data);
		if (status == ESP_OK) {
			esp_ble_gap_config_adv_data_raw((uint8_t *)&ibeacon_adv_data, sizeof(ibeacon_adv_data));
		} else {
			ESP_LOGE(TAG, "Config iBeacon data failed: %s\n", esp_err_to_name(status));
		}
	}
	xEventGroupWaitBits(ble_event_group, BLE_EVENT_ADV_DATA_RAW_SET_COMPLETE, pdFALSE, pdFALSE, portMAX_DELAY);

	xEventGroupClearBits(ble_event_group, BLE_EVENT_ADV_START_COMPLETE);
    {
        static esp_ble_adv_params_t ble_adv_params = {
            .adv_type = ADV_TYPE_NONCONN_IND,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .channel_map = ADV_CHNL_ALL,
            .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
        };
        ble_adv_params.adv_int_min = adv_int_max >> 1, // minimum advertisement interval [n * 0.625 msec]
        ble_adv_params.adv_int_max = adv_int_max,      // maximim advertisement interval [n * 0.625 msec]

        esp_ble_gap_start_advertising(&ble_adv_params);
    }
	xEventGroupWaitBits(ble_event_group, BLE_EVENT_ADV_START_COMPLETE, pdFALSE, pdFALSE, portMAX_DELAY);
	ESP_LOGI(TAG, "STARTED advertising");
}

static void
_bleStopAdv(void) {

	xEventGroupClearBits(ble_event_group, BLE_EVENT_ADV_STOP_COMPLETE);
    {
        if (esp_ble_gap_stop_advertising() != ESP_OK) {
            ESP_LOGI(TAG, "ERR stopping advertising");
        }
    }
	xEventGroupWaitBits(ble_event_group, BLE_EVENT_ADV_STOP_COMPLETE, pdFALSE, pdFALSE, portMAX_DELAY);
	ESP_LOGI(TAG, "STOPPED advertising");
}

static bleMode_t
_str2bleMode(char const * const str) {

    typedef struct {
        char * str;
        bleMode_t bleMode;
    } mode_t;
    static mode_t modes[] = {
        { "idle", BLE_MODE_IDLE},
        { "scan", BLE_MODE_SCAN},
        { "adv", BLE_MODE_ADV},
    };
    for (uint ii = 0; ii < ARRAYSIZE(modes); ii++) {
        if (strcmp(str, modes[ii].str) == 0) {
            return modes[ii].bleMode;
        }
    }
    return 0;
}

static void
_bda2devname(uint8_t const * const bda, char * const name, size_t name_len) {
	typedef struct {
		uint8_t const bda[ESP_BD_ADDR_LEN];
		char const * const name;
	} PACK8 knownBrd_t;
	static knownBrd_t knownBrds[] = {
        { {0x30, 0xAE, 0xA4, 0xCC, 0x24, 0x6A}, "esp32-1" },
        { {0x30, 0xAE, 0xA4, 0xCC, 0x32, 0x4E}, "esp32-2" },
        { {0xAC, 0x67, 0xB2, 0x53, 0x82, 0x8A}, "esp32-3" },
        { {0xAC, 0x67, 0xB2, 0x53, 0x7F, 0x22}, "esp32-4" },
        { {0xAC, 0x67, 0xB2, 0x53, 0x84, 0x82}, "esp32-5" },
        { {0xAC, 0x67, 0xB2, 0x53, 0x84, 0xAA}, "esp32-6" },
        { {0x24, 0x0A, 0xC4, 0xEB, 0x36, 0x8A}, "esp32-7" },
        { {0xAC, 0x67, 0xB2, 0x53, 0x93, 0x1E}, "esp32-8" },
        { {0xAC, 0x67, 0xB2, 0x53, 0x84, 0xB2}, "esp32-9" },
        { {0xAC, 0x67, 0xB2, 0x53, 0x7B, 0x3A}, "esp32-10" },
        { {0x8c, 0xaa, 0xb5, 0x85, 0x0a, 0x7e}, "esp32-11" },
        { {0x8c, 0xaa, 0xb5, 0x86, 0x2b, 0xa2}, "esp32-12" },
        { {0x8c, 0xaa, 0xb5, 0x86, 0x22, 0xc2}, "esp32-13" },
        { {0x8c, 0xaa, 0xb5, 0x85, 0x43, 0x42}, "esp32-14" },
        { {0x8c, 0xaa, 0xb5, 0x85, 0x6d, 0x06}, "esp32-15" },
        { {0x8c, 0xaa, 0xb5, 0x85, 0x05, 0xf2}, "esp32-16" },
        { {0x8c, 0xaa, 0xb5, 0x84, 0xe9, 0x76}, "esp32-17" },
        { {0x8c, 0xaa, 0xb5, 0x86, 0x2d, 0x5a}, "esp32-18" },
        { {0x8c, 0xaa, 0xb5, 0x84, 0xec, 0xc6}, "esp32-19" },
        { {0x8c, 0xaa, 0xb5, 0x86, 0x08, 0x46}, "esp32-20" },
        { {0x30, 0xae, 0xa4, 0xcc, 0x45, 0x06}, "esp32-wrover-1" }
	};
	for (uint ii=0; ii < ARRAYSIZE(knownBrds); ii++) {
		if (memcmp(bda, knownBrds[ii].bda, ESP_BD_ADDR_LEN) == 0) {
			strncpy(name, knownBrds[ii].name, name_len);
			return;
		}
	}
	snprintf(name, name_len, "esp32_%02x%02x",
			 bda[ESP_BD_ADDR_LEN-2], bda[ESP_BD_ADDR_LEN-1]);
}

static uint
_splitArgs(char * data, char * args[], uint const args_len) {

    uint ii = 0;
    char const * const delim = " ";
    char * save;
    char * p = strtok_r(data, delim, &save);
    while (p && ii < args_len) {
        args[ii++] = p;
        p = strtok_r(NULL, delim, &save);
    }
    return ii;
}

static bleMode_t
_changeBleMode(bleMode_t const current, bleMode_t const new, uint16_t const adv_int_max) {

    if (new == current) {
        return new;
    }
    switch(new) {
        case BLE_MODE_IDLE:
            if (current == BLE_MODE_SCAN) _bleStopScan();
            if (current == BLE_MODE_ADV) _bleStopAdv();
            break;
        case BLE_MODE_SCAN:
            if (current == BLE_MODE_ADV) _bleStopAdv();
            _bleStartScan(adv_int_max + 0x04);
            break;
        case BLE_MODE_ADV:
            if (current == BLE_MODE_SCAN) _bleStopScan();
            _bleStartAdv(adv_int_max);
            break;
    }
    return new;
}

void
ble_scan_task(void * ipc_void) {

	ipc = ipc_void;

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	esp_bt_controller_init(&bt_cfg);
	esp_bt_controller_enable(ESP_BT_MODE_BLE);
	_initIbeacon();

    // first  message to ipc->toMqttQ is the MAC address (rx'ed by mqtt_client_task)

    uint8_t const * const bda = esp_bt_dev_get_address();
    {
        char * const bdaStr = malloc(BLE_DEVMAC_LEN);
        toMqttMsg_t msg = {
            .dataType = TO_MQTT_MSGTYPE_DEVMAC,
            .data = _bla2str(bda, bdaStr)
        };
        if (xQueueSendToBack(ipc->toMqttQ, &msg, 0) != pdPASS) {
            ESP_LOGE(TAG, "toMqttQ full (1st)");  // should never happen, since its the first msg
            free(msg.data);
        }
	}

	// second message to ipc->toMqttQ is the device name (rx'ed by mqtt_client_task)

	char devName[BLE_DEVNAME_LEN];
	_bda2devname(bda, devName, BLE_DEVNAME_LEN);
    {
        toMqttMsg_t msg = {
            .dataType = TO_MQTT_MSGTYPE_DEVNAME,
            .data = strdup(devName)
        };
        if (xQueueSendToBack(ipc->toMqttQ, &msg, 0) != pdPASS) {
            ESP_LOGE(TAG, "toMqttQ full (2nd)");  // should never happen, since its the first msg
            free(msg.data);
        }
    }

	ble_event_group = xEventGroupCreate();  // for event handler to signal completion

    uint16_t adv_int_max = (30 << 4) / 10;  // 30 msec  [n * 0.625 msec]
    bleMode_t bleMode = _changeBleMode(BLE_MODE_IDLE, BLE_MODE_ADV, adv_int_max);

	while (1) {

		fromMqttMsg_t msg;
		if (xQueueReceive(ipc->fromMqttQ, &msg, (TickType_t)(1000L / portTICK_PERIOD_MS)) == pdPASS) {

            if (msg.dataType != FROM_MQTT_MSGTYPE_CTRL) {
                ESP_LOGE(TAG, "dataType (%d) err", msg.dataType);
            } else {
                char * args[3];
                uint8_t argc = _splitArgs(msg.data, args, ARRAYSIZE(args));

                if (strcmp(args[0], "int") == 0 && argc >= 2) {

                        // args[1] should be in 10s of msec, e.g. for 150 msec, specify 15
                        adv_int_max = (atoi(args[1]) << 4) / 10;

                        bleMode_t const orgBleMode = bleMode;  // args[1] in msec
                        bleMode = _changeBleMode(bleMode, BLE_MODE_IDLE, adv_int_max);
                        bleMode = _changeBleMode(bleMode, orgBleMode, adv_int_max);

                } else {

                    bleMode_t const newBleMode = _str2bleMode(args[0]);
                    if (newBleMode) {
                        bleMode = _changeBleMode(bleMode, newBleMode, adv_int_max);
                    }
                }
            }
			free(msg.data);
		}
	}
}
