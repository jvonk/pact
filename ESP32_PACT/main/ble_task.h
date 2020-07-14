#pragma once

void ble_task(void * ipc_void);

void bleMac2devName(uint8_t const * const bda, char * const name, size_t name_len);
char * bleMac2str(uint8_t const * const bda, char * const str);