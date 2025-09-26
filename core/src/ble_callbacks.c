#include "ble_callbacks.h"
#include "esp_log.h"

#define TAG "BLE_CB"

// These globals are defined in main.c as extern
extern uint16_t char_handle;
extern uint16_t conn_id;
extern esp_gatt_if_t gatts_if;
extern bool notify_enabled;

void gatts_event_handler(esp_gatts_cb_event_t event,
                         esp_gatt_if_t gatts_if_param,
                         esp_ble_gatts_cb_param_t *param)
{
    switch(event){
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATT server registered");
            break;

        case ESP_GATTS_CONNECT_EVT:
            conn_id = param->connect.conn_id;
            gatts_if = gatts_if_param;
            ESP_LOGI(TAG, "Client connected: conn_id=%d", conn_id);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Client disconnected");
            notify_enabled = false;
            break;

        case ESP_GATTS_WRITE_EVT:
            // Enable/disable notifications if client writes to CCCD
            if(param->write.handle == char_handle + 1){ // CCCD
                uint16_t value = param->write.value[0] | (param->write.value[1]<<8);
                notify_enabled = (value == 1);
                ESP_LOGI(TAG, "Notify %s", notify_enabled ? "enabled" : "disabled");
            }
            break;

        default:
            break;
    }
}
