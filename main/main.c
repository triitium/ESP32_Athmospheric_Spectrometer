#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"

#define TAG "TCD_ADS_MODULAR"

// ---------------- Configuration ----------------
#define PIXELS 3648
#define SAMPLE_SIZE 10
#define SPI_CLOCK_HZ 1000000  // 1 MHz
#define BLE_MTU 512           // bytes per notification
#define MEASUREMENT_INTERVAL_MS (15*60*1000) // 15 minutes

#define DEVICE_NAME "ESP32_TCD1304"
#define GATTS_SERVICE_UUID 0x00FF
#define GATTS_CHAR_UUID    0xFF01

// ---------------- Pin definitions ----------------
#define PIN_MCLK   GPIO_NUM_12
#define PIN_SH     GPIO_NUM_13
#define PIN_ICG    GPIO_NUM_14
#define PIN_SI     GPIO_NUM_15
#define PIN_SPI_SCLK GPIO_NUM_18
#define PIN_SPI_MISO GPIO_NUM_19
#define PIN_SPI_CS   GPIO_NUM_5

#define SPI_HOST_USED HSPI_HOST

// ---------------- Globals ----------------
static spi_device_handle_t spi_adc = NULL;
static QueueHandle_t raw_line_queue;
static QueueHandle_t avg_line_queue;

static uint16_t char_handle = 0;
static uint16_t conn_id = 0;
static esp_gatt_if_t gatts_if = 0;
static bool notify_enabled = false;

// ---------------- GPIO + MCLK Initialization ----------------
static void init_gpio(void) {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<PIN_MCLK)|(1ULL<<PIN_SH)|(1ULL<<PIN_ICG)|(1ULL<<PIN_SI)
    };
    gpio_config(&io_conf);
}

static void init_mclk(void) {
    ledc_timer_config_t timer_conf = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 1000000, // 1 MHz CCD MCLK
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .channel = LEDC_CHANNEL_0,
        .gpio_num = PIN_MCLK,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 4095
    };
    ledc_channel_config(&channel_conf);
}

// ---------------- SPI ADC Initialization ----------------
static void init_spi_adc(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_SPI_MISO,
        .mosi_io_num = -1,
        .sclk_io_num = PIN_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_USED, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = PIN_SPI_CS,
        .queue_size = 1
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_USED, &devcfg, &spi_adc));
}

// ---------------- ADC read ----------------
static inline uint16_t ads7886_read16(void) {
    uint8_t tx[3] = {0,0,0}, rx[3] = {0};
    spi_transaction_t t = {
        .length = 24,
        .tx_buffer = tx,
        .rx_buffer = rx
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi_adc, &t));
    return (rx[1] << 8) | rx[2];
}

// ---------------- Acquisition Task ----------------
static void acquisition_task(void *arg) {
    while(1) {
        for(int s = 0; s < SAMPLE_SIZE; s++){
            uint16_t line[PIXELS];

            // Trigger CCD
            gpio_set_level(PIN_SI, 1);
            ets_delay_us(1);
            gpio_set_level(PIN_SI, 0);

            for(int i = 0; i < PIXELS; i++){
                gpio_set_level(PIN_SH, 1);
                ets_delay_us(0);
                line[i] = ads7886_read16();
                gpio_set_level(PIN_SH, 0);
            }

            // Push line to averaging queue
            xQueueSend(raw_line_queue, line, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(10)); // small delay between lines
        }

        // Wait until next measurement period
        vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_INTERVAL_MS));
    }
}

// ---------------- Averaging Task ----------------
static void averaging_task(void *arg) {
    uint16_t lines[SAMPLE_SIZE][PIXELS];
    uint32_t accum[PIXELS];
    uint16_t avg_line[PIXELS];
    int idx = 0;
    int count = 0;

    while(1) {
        uint16_t new_line[PIXELS];
        xQueueReceive(raw_line_queue, new_line, portMAX_DELAY);

        memcpy(lines[idx], new_line, sizeof(new_line));
        idx = (idx + 1) % SAMPLE_SIZE;
        if(count < SAMPLE_SIZE) count++;

        memset(accum, 0, sizeof(accum));
        for(int s = 0; s < count; s++)
            for(int i = 0; i < PIXELS; i++)
                accum[i] += lines[s][i];

        for(int i = 0; i < PIXELS; i++)
            avg_line[i] = accum[i] / count;

        xQueueSend(avg_line_queue, avg_line, portMAX_DELAY);
    }
}

// ---------------- BLE Task ----------------
static void ble_task(void *arg) {
    uint16_t avg_line[PIXELS];
    uint8_t buffer[BLE_MTU];

    while(1) {
        xQueueReceive(avg_line_queue, avg_line, portMAX_DELAY);

        if(!notify_enabled) continue;

        size_t bytes_to_send = PIXELS*2;
        size_t offset = 0;

        while(offset < bytes_to_send){
            size_t chunk = (bytes_to_send - offset) > BLE_MTU ? BLE_MTU : (bytes_to_send - offset);
            for(size_t i = 0; i < chunk/2; i++){
                buffer[i*2]   = avg_line[offset/2 + i] & 0xFF;
                buffer[i*2+1] = (avg_line[offset/2 + i] >> 8) & 0xFF;
            }
            esp_ble_gatts_send_indicate(gatts_if, conn_id, char_handle, chunk, buffer, false);
            offset += chunk;
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

// ---------------- BLE Initialization ----------------
static void ble_init(void){
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    esp_ble_gatts_register_callback(gatts_event_handler); // implement separately
    esp_ble_gap_set_device_name(DEVICE_NAME);
}

// ---------------- Main ----------------
void app_main(void) {
    init_gpio();
    init_mclk();
    init_spi_adc();

    raw_line_queue = xQueueCreate(SAMPLE_SIZE, sizeof(uint16_t)*PIXELS);
    avg_line_queue = xQueueCreate(1, sizeof(uint16_t)*PIXELS);

    ble_init();

    // Create modular tasks
    xTaskCreate(acquisition_task, "acquisition_task", 4096, NULL, 10, NULL);
    xTaskCreate(averaging_task,   "averaging_task",   4096, NULL, 9,  NULL);
    xTaskCreate(ble_task,         "ble_task",         8192, NULL, 8,  NULL);
}
