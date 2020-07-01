/**
 * @brief BLE iBeacon advertisement or scanning
 **/
// Copyright © 2020, Johan Vonk
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

#define ARRAYSIZE(a) (sizeof(a) / sizeof(*(a)))
#define ALIGN( type ) __attribute__((aligned( __alignof__( type ) )))
#define PACK( type )  __attribute__((aligned( __alignof__( type ) ), packed ))
#define PACK8  __attribute__((aligned( __alignof__( uint8_t ) ), packed ))

static const char *TAG = "ble_scan_task";
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

typedef enum {
	BLE_MODE_IDLE = 1,
	BLE_MODE_SCAN,
	BLE_MODE_ADV
} bleMode_t;

bleMode_t _bleMode = BLE_MODE_IDLE;

extern esp_ble_ibeacon_vendor_t vendor_config;

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {

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
		esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
		switch (scan_result->scan_rst.search_evt) {
            case ESP_GAP_SEARCH_INQ_RES_EVT:
                if (esp_ble_is_ibeacon_packet(scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len)) {

                    // format iBeacon scan result as JSON and forward to ipc->measurementQ

                    esp_ble_ibeacon_t const * const ibeacon_data = (esp_ble_ibeacon_t *)(scan_result->scan_rst.ble_adv);

                    uint len = 0;
                    char payload[256];

                    len += sprintf(payload + len, "{ \"Address\": \"");
                    for (uint ii = 0; ii < ESP_BD_ADDR_LEN; ii++) {
                        len += sprintf(payload + len, "%02x%c", scan_result->scan_rst.bda[ii], (ii < ESP_BD_ADDR_LEN - 1) ? ':' : '"');
                    }
#if 0
                    len += sprintf(payload + len, "\", \"UUID\": \"");
                    for (uint ii = 0; ii < ESP_UUID_LEN_128; ii++) {
                        if (ii == 4 || ii == 6 || ii == 8) *(payload + len++) = '-';
                        len += sprintf(payload + len, "%02x", ibeacon_data->ibeacon_vendor.proximity_uuid[ii]);
                    }
                    uint16_t major = ENDIAN_CHANGE_U16(ibeacon_data->ibeacon_vendor.major);
                    uint16_t minor = ENDIAN_CHANGE_U16(ibeacon_data->ibeacon_vendor.minor);
                    len += sprintf(payload + len, "\", \"Major\": \"%04x\"", major);
                    len += sprintf(payload + len, "\", \"Minor\": \"%04x\"", minor);
#endif
                    len += sprintf(payload + len, ", \"txPwr\": %d", ibeacon_data->ibeacon_vendor.measured_power);
                    len += sprintf(payload + len, ", \"RSSI\": %d }", scan_result->scan_rst.rssi);

                    //ESP_LOGI(TAG, "body = \"%.*s\"", len, payload);
                    char * const msg = strdup(payload);
                    //ESP_LOGI(TAG, "measurementQ Tx: \"%s\"", msg);
                    if (xQueueSendToBack(ipc->measurementQ, &msg, 0) != pdPASS) {
                        ESP_LOGW(TAG, "measurementQ full");
                        free(msg);
                    }
    #if 0
                } else {
                    if (scan_result->scan_rst.ble_adv) {
                        static esp_ble_ibeacon_head_t const ibeacon_common_head = {
                            .flags = {0x02, 0x01, 0x06},
                            .length = 0x1A,
                            .type = 0xFF,
                            .company_id = 0x004C,
                            .beacon_type = 0x1502 };
                        bool lenOk = scan_result->scan_rst.adv_data_len == 0x1E;
                        bool headOk = memcmp(scan_result->scan_rst.ble_adv, (uint8_t *)&ibeacon_common_head, sizeof(ibeacon_common_head)) == 0;
                        ESP_LOGI(TAG, "len = 0x%02x (%s), head = 0x%02x%02x%02x%02x%02x%02x%02x%02x (%s)",
                                scan_result->scan_rst.adv_data_len, lenOk ? "OK" : "ERR",
                                scan_result->scan_rst.ble_adv[0], scan_result->scan_rst.ble_adv[1],
                                scan_result->scan_rst.ble_adv[2], scan_result->scan_rst.ble_adv[3],
                                scan_result->scan_rst.ble_adv[4], scan_result->scan_rst.ble_adv[5],
                                scan_result->scan_rst.ble_adv[6], scan_result->scan_rst.ble_adv[7],
                                headOk ? "OK" : "ERR");
                    }
    #endif
                }
                break;
            default:
                break;
            }
            break;
        }
        default:
            break;
	}
}

void ble_ibeacon_appRegister(void)  // register the scan callback function to the gap module
{
	ESP_LOGI(TAG, "register callback");

	esp_err_t status = esp_ble_gap_register_callback(esp_gap_cb);
	if (status != ESP_OK) ESP_LOGE(TAG, "gap register error: %s", esp_err_to_name(status));
}

void ble_ibeacon_init(void) {
	esp_bluedroid_init();
	esp_bluedroid_enable();
	ble_ibeacon_appRegister();
}

static void _bleStartScan(void) {
	xEventGroupClearBits(ble_event_group, BLE_EVENT_SCAN_START_COMPLETE);
	uint32_t duration = 0;  // [sec], 0 means scan permanently
	esp_ble_gap_start_scanning(duration);
	xEventGroupWaitBits(ble_event_group, BLE_EVENT_SCAN_START_COMPLETE, pdFALSE, pdFALSE, portMAX_DELAY);
	ESP_LOGI(TAG, "STARTED scanning");
}

static void _bleStopScan(void) {
	if (_bleMode != BLE_MODE_SCAN) return;
	xEventGroupClearBits(ble_event_group, BLE_EVENT_SCAN_STOP_COMPLETE);

	if (esp_ble_gap_stop_scanning() != ESP_OK) {
		ESP_LOGI(TAG, "ERR stopping scanning");
	}
	xEventGroupWaitBits(ble_event_group, BLE_EVENT_SCAN_STOP_COMPLETE, pdFALSE, pdFALSE, portMAX_DELAY);
	ESP_LOGI(TAG, "STOPPED scanning");
}

static void _bleStartAdv(void) {
	if (_bleMode != BLE_MODE_ADV) return;
	xEventGroupClearBits(ble_event_group, BLE_EVENT_ADV_START_COMPLETE);

    static esp_ble_adv_params_t ble_adv_params = {
        .adv_int_min = 0x00F0,  // minimum advertisement interval 150 msec [n * 0.625 msec]
        .adv_int_max = 0x01E0,  // maximim advertisement interval 300 msec [n * 0.625 msec]
        .adv_type = ADV_TYPE_NONCONN_IND,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };

	esp_ble_gap_start_advertising(&ble_adv_params);
	xEventGroupWaitBits(ble_event_group, BLE_EVENT_ADV_START_COMPLETE, pdFALSE, pdFALSE, portMAX_DELAY);
	ESP_LOGI(TAG, "STARTED advertising");
}

static void _bleStopAdv(void) {
	xEventGroupClearBits(ble_event_group, BLE_EVENT_ADV_STOP_COMPLETE);

	if (esp_ble_gap_stop_advertising() != ESP_OK) {
		ESP_LOGI(TAG, "ERR stopping advertising");
	}

	EventBits_t bits = xEventGroupWaitBits(ble_event_group, BLE_EVENT_ADV_STOP_COMPLETE, pdFALSE, pdFALSE, portMAX_DELAY);
	if (!bits) esp_restart();  // give up

	ESP_LOGI(TAG, "STOPPED advertising");
}

static bleMode_t _str2bleMode(char const * const msg) {

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
        if (strcmp(msg, modes[ii].str) == 0) {
            return modes[ii].bleMode;
        }
    }
    return 0;
}

void _bda2name(uint8_t const * const bda, char * const name, size_t name_len) {
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

void _bda2str(uint8_t const * const bda, char * const str, size_t str_len) {

    uint len = 0;
    for (uint ii = 0; ii < ESP_BD_ADDR_LEN; ii++) {
        //ESP_LOGI(TAG, "ii %d, str_len - len = %d", ii, str_len - len);
        len += snprintf(str + len, str_len - len, "0x%02x", bda[ii]);
        if (ii < ESP_BD_ADDR_LEN - 1) {
            str[len++] = ',';
            str[len++] = ' ';
        }
    }
}

void ble_scan_task(void * ipc_void) {

	ipc = ipc_void;

	ESP_LOGI(TAG, "start");
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	esp_bt_controller_init(&bt_cfg);
	esp_bt_controller_enable(ESP_BT_MODE_BLE);
	ble_ibeacon_init();

	// first message to ipc->measurementQ is the BLE MAC address (rx'ed by mqtt_client_task

	uint8_t const * const myBda = esp_bt_dev_get_address();
    char brdName[40];
    _bda2str(myBda, brdName, ARRAYSIZE(brdName));
    ESP_LOGI(TAG, "BD_ADDR = {%s}", brdName);

    uint const msg_len = 32;
	char * msg = malloc(msg_len);
	_bda2name(myBda, msg, msg_len);

	ESP_LOGI(TAG, "measurementQ Tx: \"%s\"", msg);
	if (xQueueSendToBack(ipc->measurementQ, &msg, 0) != pdPASS) {
		ESP_LOGW(TAG, "measurementQ full");  // should never happen here, since its the first msg
		free(msg);
	}

	ble_event_group = xEventGroupCreate();  // lets event handler signal completion

	// get ready for advertising

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

	// get ready for scanning

	xEventGroupClearBits(ble_event_group, BLE_EVENT_SCAN_PARAM_SET_COMPLETE);
	{
        static esp_ble_scan_params_t ble_scan_params = {
            .scan_type = BLE_SCAN_TYPE_ACTIVE,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
            .scan_interval = 0x50, // time between start of scans 50 msec [n * 0.625 msec]
            .scan_window = 0x30,   // scan duration 30 msec [n * 0.625 msec]
            .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};

		esp_ble_gap_set_scan_params(&ble_scan_params);
	}
	xEventGroupWaitBits(ble_event_group, BLE_EVENT_SCAN_PARAM_SET_COMPLETE, pdFALSE, pdFALSE, portMAX_DELAY);

	_bleMode = BLE_MODE_ADV;
	_bleStartAdv();

	while (1) {
		char * msg;
		if (xQueueReceive(ipc->controlQ, &msg, (TickType_t)(1000L / portTICK_PERIOD_MS)) == pdPASS) {
			// ESP32 can only do one function at a time (SCAN || ADVERTISE)

            if (strcmp(msg, "restart")) {
                esp_restart();
            } else if (strcmp(msg, "echo")) {
                char * const reply = strdup(msg);
                if (xQueueSendToBack(ipc->measurementQ, &reply, 0) != pdPASS) {
                    ESP_LOGW(TAG, "measurementQ full");
                    free(reply);
                }
            } else {
                bleMode_t const bleMode = _str2bleMode(msg);
                //ESP_LOGI(TAG, "ctrl msg \"%s\", new bleMode = %d (was %d)", msg, bleMode, _bleMode);
                if (bleMode && bleMode != _bleMode) {
                    switch(bleMode) {
                        case BLE_MODE_IDLE:
                            _bleStopScan();
                            _bleStopAdv();
                            break;
                        case BLE_MODE_SCAN:
                            _bleStopAdv();
                            _bleStartScan();
                            break;
                        case BLE_MODE_ADV:
                            _bleStopScan();
                            _bleStartAdv();
                            break;
                    }
                    _bleMode = bleMode;
                }
            }
			free(msg);
		}
	}
}
