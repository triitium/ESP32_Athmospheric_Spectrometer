#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ---------- Constants ----------
#define PIXELS 3648
#define SAMPLE_SIZE 10
#define BLE_MTU 512
#define MEASUREMENT_INTERVAL_MS (15*60*1000)

// ---------- GPIO Pins ----------
#define PIN_MCLK   12
#define PIN_SH     13
#define PIN_ICG    14
#define PIN_SI     15
#define PIN_SPI_SCLK 18
#define PIN_SPI_MISO 19
#define PIN_SPI_CS   5

// ---------- Queues ----------
extern QueueHandle_t raw_line_queue;
extern QueueHandle_t avg_line_queue;

// ---------- BLE Globals ----------
extern uint16_t char_handle;
extern uint16_t conn_id;
extern bool notify_enabled;

// ---------- Functions ----------
void init_gpio(void);
void init_mclk(void);
void init_spi_adc(void);
uint16_t ads7886_read16(void);
