#pragma once

#define BLE_DEVNAME_LEN (32)
#define BLE_DEVMAC_LEN (6 * 3)

typedef struct ipc_t {
    QueueHandle_t toBleQ;
    QueueHandle_t toMqttQ;
    struct dev {
        char mac[BLE_DEVMAC_LEN];
        char name[BLE_DEVNAME_LEN];
    } dev;

} ipc_t;

typedef enum toMqttMsgType_t {
    TO_MQTT_MSGTYPE_DATA,
} toMqttMsgType_t;

typedef struct toMqttMsg_t {
    toMqttMsgType_t dataType;
    char * data;  // must be freed by recipient
} toMqttMsg_t;

typedef enum toBleMsgType_t {
    TO_BLE_MSGTYPE_CTRL
} toBleMsgType_t;

typedef struct toBleMsg_t {
    toBleMsgType_t dataType;
    char * data;  // must be freed by recipient
} toBleMsg_t;

void sendToBle(toBleMsgType_t const dataType, char const * const data, ipc_t const * const ipc);
void sendToMqtt(toMqttMsgType_t const dataType, char const * const data, ipc_t const * const ipc);