# VANGUARD-PASIL Ground Control Station (GCS)

The Ground Control Station (GCS) is a split-architecture counterpart to the VANGUARD-PASIL Flight Controller. It reads pilot inputs via Hall-effect joysticks, transmits telemetry via a 2.4 GHz RF link, and displays real-time flight data on a 2.4" TFT screen without blocking critical transmission loops.

## 💻 Hardware Overview

* **Microcontroller:** ESP32 DevKit (Xtensa LX6 @ 240 MHz).
* **Telemetry:** NRF24L01+PA+LNA operating at 2 Mbps.
* **Display:** 2.4" SPI ILI9341 TFT Display.
* **Inputs:** 2x Hall-effect joysticks read via ADC1.
* **OS:** FreeRTOS (dedicated task drives the TFT at 20 Hz).

## 🔧 TFT_eSPI Configuration (`User_Setup.h`)

To ensure the `TFT_eSPI` library interfaces correctly with the 2.4" ILI9341 display driven by the ESP32[cite: 1], you must modify your `User_Setup.h` (or custom user config file) as follows. 

Comment out the default driver and strictly define the ILI9341 alongside standard ESP32 VSPI hardware pins.
```cpp
// ================================================================
// VANGUARD-PASIL GCS - TFT_eSPI Configuration
// ================================================================

// 1. Define the specific display driver
#define ILI9341_DRIVER       // Select the ILI9341 driver

// 2. Define the ESP32 VSPI Pins
// (Adjust these if your custom GCS schematic routes SPI differently)
#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS    5  // Chip select control pin
#define TFT_DC    2  // Data Command control pin
#define TFT_RST   4  // Reset pin (could connect to RST pin)

// 3. Fonts (Optional but recommended for GCS UI)
#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font

// 4. SPI Frequency (Optimised for FreeRTOS 20Hz update rate)
#define SPI_FREQUENCY  40000000
```


## 📡 RF Payload Structure

The ESP32 transmits a packed 32-byte binary payload via the NRF24L01:

16 bytes: 4 floats (Roll, Pitch, Yaw, Throttle).

1 byte: Arming state.

1 byte: Sequence counter.

2 bytes: CRC-16 (CCITT).

12 bytes: Reserved for future telemetry expansions.
