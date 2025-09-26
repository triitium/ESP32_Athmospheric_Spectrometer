# ESP32 Athmospheric Spectrometer

This project implements a modular ESP32 firmware to interface with a **TCD1304 linear CCD** sensor, read analog values via an SPI ADC, perform averaging, and send the results via **BLE notifications**. The architecture is designed for **future expansion** and clean modularity.

---

## **Features**

* Continuous acquisition of CCD lines (3648 pixels per line).
* Averaging over a configurable number of samples (`SAMPLE_SIZE`).
* BLE notifications with automatic chunking according to `BLE_MTU`.
* Modular task architecture:

  * **Acquisition Task:** Reads CCD via SPI.
  * **Averaging Task:** Maintains a rolling average of lines.
  * **BLE Task:** Sends averaged data to connected clients.
* Configurable pins for SPI and control signals (MCLK, SI, SH, ICG).
* Fully compatible with **PlatformIO** and **ESP-IDF**.

---

## **Project Structure**

```
ESP32_TCD1304/
├── sys/
│   ├── src/
│   │   ├── main.c
│   │   └── ble_callbacks.c
│   ├── inc/
│   │   ├── main.h
│   │   └── ble_callbacks.h
│   └── CMakeLists.txt
├── platformio.ini
├── sdkconfig
└── CMakeLists.txt
```

* `sys/src/` – Source code for tasks and BLE callbacks.
* `sys/inc/` – Public headers for sharing globals and function declarations.

---

## **Hardware Connections**

| Signal   | ESP32 Pin |
| -------- | --------- |
| MCLK     | 12        |
| SI       | 15        |
| SH       | 13        |
| ICG      | 14        |
| SPI SCLK | 18        |
| SPI MISO | 19        |
| SPI CS   | 5         |

---

## **Configuration**

Edit `main.h` for:

* `PIXELS` – number of pixels per CCD line.
* `SAMPLE_SIZE` – number of lines to average.
* `BLE_MTU` – maximum bytes per BLE notification.

`platformio.ini` provides PlatformIO configuration:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = espidf
monitor_speed = 115200
build_flags =
    -D BLE_MTU=512
```

---

## **Build and Upload**

Using PlatformIO:

```bash
pio run -t upload
pio device monitor
```

Using ESP-IDF:

```bash
idf.py build
idf.py flash monitor
```

---

## **Working Principle**

1. The ESP32 will continuously acquire lines, average them, and expose BLE notifications.
2. Any BLE client can subscribe to receive the averaged data.

**Note:** Lost data is irrelevant; the system does not buffer past results beyond the averaging window.

---

## **License**

MIT License. Free to use and modify.
