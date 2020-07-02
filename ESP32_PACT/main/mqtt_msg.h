#pragma once

typedef enum toMqttMsgType_t {
    TO_MQTT_MSGTYPE_CTRL,
    TO_MQTT_MSGTYPE_DATA,
    TO_MQTT_MSGTYPE_DEVMAC,
    TO_MQTT_MSGTYPE_DEVNAME,
} toMqttMsgType_t;

typedef struct toMqttMsg_t {
    toMqttMsgType_t dataType;
    char * data;  // must be freed by recipient
} toMqttMsg_t;

typedef enum fromMqttMsgType_t {
    FROM_MQTT_MSGTYPE_CTRL
} fromMqttMsgType_t;

typedef struct fromMqttMsg_t {
    fromMqttMsgType_t dataType;
    char * data;  // must be freed by recipient
} fromMqttMsg_t;

#define BLE_DEVNAME_LEN (32)
#define BLE_DEVMAC_LEN (6 * 3)