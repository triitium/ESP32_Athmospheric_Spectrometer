#pragma once
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include <stdbool.h>
#include <stdint.h>

extern uint16_t char_handle;
extern uint16_t conn_id;
extern esp_gatt_if_t gatts_if;
extern bool notify_enabled;

void gatts_event_handler(esp_gatts_cb_event_t event,
                         esp_gatt_if_t gatts_if_param,
                         esp_ble_gatts_cb_param_t *param);
