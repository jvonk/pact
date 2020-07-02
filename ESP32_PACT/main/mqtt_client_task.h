/* Copyright © 2020, Johan Vonk */
/* SPDX-License-Identifier: MIT */
#pragma once

typedef struct {
    QueueHandle_t toMqttQ;
    QueueHandle_t fromMqttQ;
} mqtt_client_task_ipc_t;

void mqtt_client_task(void * ipc_void);

