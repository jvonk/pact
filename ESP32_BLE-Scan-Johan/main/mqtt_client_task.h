/* Copyright © 2020, Johan Vonk */
/* SPDX-License-Identifier: MIT */
#pragma once

typedef struct {
    QueueHandle_t measurementQ;
    QueueHandle_t controlQ;
} mqtt_client_task_ipc_t;

void mqtt_client_task(void * ipc_void);

