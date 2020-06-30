/* Copyright © 2020, Johan Vonk */
/* SPDX-License-Identifier: MIT */
#pragma once

typedef struct {
    QueueHandle_t measurementQ;
    QueueHandle_t controlQ;
} ble_scan_task_ipc_t;

void ble_scan_task(void * ipc);