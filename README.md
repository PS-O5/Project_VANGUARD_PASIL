# VANGUARD-PASIL

**Flight Control Architecture & Integration**  
*Document Revision: E (Production-Ready)*

VANGUARD-PASIL is a real-time, safety-critical flight control testbed that bridges human pilot commands to physical motor actuation. Built on Apache NuttX 12.13.0, this platform evolves from a custom bare-metal kernel to a fully telemetric split-architecture system. 

## 🚀 Key Features

* **Deterministic Control Loop:** Runs a single high-priority thread (255) at 500 Hz ($\Delta t=0.002~s$) for consistent execution.
* **CMSIS-DSP Acceleration:** Hardware FPU acceleration computes a 7x7 Extended Kalman Filter (EKF) to maintain optimal attitude and gyro bias estimates.
* **Multi-Stage Failsafe:** Safety-critical protection forces zero throttle and attitude levelling upon RF link loss, transitioning to a hard lock/motor stop after 2 seconds of lost connection.
* **Clean Power Architecture:** Utilizes a star-ground topology and isolated regulators (including a dedicated ultra-low-noise 3.3V LDO for the RF module) to prevent ESC noise from causing brownouts.
* **Application-Level Sensor Management:** Custom I2C management bypasses kernel conflicts, including forced-mode polling for the BMP280 barometer[cite: 1].

## 🛠️ Hardware Stack

### Flight Controller (Airborne)
* **MCU:** STM32F411CEU6 "BlackPill" @ 100 MHz.
* **IMU:** BMI160.
* **Magnetometer:** QMC5883P.
* **Barometer:** BMP280.
* **Time of Flight (ToF):** VL53L0X.
* **Actuation:** 4x ESCs driven via deterministic 50 Hz hardware PWM (TIM2/TIM3).

### Ground Control Station (GCS)
* **MCU:** ESP32 DevKit (Dual-core Xtensa LX6 @ 240 MHz).
* **Inputs:** 2x Hall-effect joysticks on ADC1.
* **Display:** 2.4" SPI ILI9341 TFT.

### Telemetry Link
* **Module:** Full-duplex NRF24L01+PA+LNA (2.4 GHz).
* **Performance:** 2 Mbps air data rate with Auto-ACK enabled.

## ⚙️ Build & Recovery Instructions

1. **Clean Build:** Execute `make clean` after any Kconfig changes.
2. **FPU:** Ensure `CONFIG_LIBC_FLOATINGPOINT=y` is enabled.
3. **Sensors:** Disable all kernel sensor drivers except the BMI160.
4. **CMSIS:** Verify the Board Makefile lists required CMSIS source files.
5. **Failsafe Tuning:** Timeout constants are configurable in `pasil_ekf.h`.
