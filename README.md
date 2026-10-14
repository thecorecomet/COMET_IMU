# COMET_IMU

[![Arduino IDE Compatible](https://img.shields.io/badge/Arduino%20IDE-Compatible-00979D.svg?logo=arduino)](https://www.arduino.cc/)
[![PlatformIO Compatible](https://img.shields.io/badge/PlatformIO-Compatible-f18c34.svg?logo=platformio)](https://platformio.org/)
[![Architectures](https://img.shields.io/badge/Architectures-ESP32%20%7C%20STM32%20%7C%20AVR%20%7C%20RP2040-blue.svg)](#cross-platform-architecture)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A unified, high-performance C++ IMU library featuring **automatic bus detection (I2C & SPI)**, **on-chip hardware sensor fusion offloading**, fallback **Madgwick, Mahony, Kalman, and Complementary 6/9-DoF AHRS filters**, and **concurrent multi-IMU orchestration**. Designed for **ESP32**, **STM32**, and **classic Arduino** microcontrollers.

- **Author**: Bhushan Patil ([bhushanshampatil@gmail.com](mailto:bhushanshampatil@gmail.com))
- **Repository**: [https://github.com/thecorecomet/COMET_IMU.git](https://github.com/thecorecomet/COMET_IMU.git)

---

## 🚀 Key Features

* **🔍 1-Line Auto-Detection**:
  Just call `imu.autoDetect(&Wire)` or `imu.autoDetectSPI(csPin)`. The library scans candidate addresses, queries `WHO_AM_I` registers, and automatically binds the proper driver with optimal configuration.
* **🧠 Inbuilt Hardware Sensor Fusion Offload**:
  Directly reads fused Quaternions, Linear Acceleration, and Gravity from sensors with on-chip processors (e.g., **Bosch BNO055** with internal Cortex-M0 and **BNO080/085** SH-2 sensor hub), utilizing **zero host MCU compute cycles**!
* **⚡ 4 High-Speed Software Fusion Filters**:
  For standard raw IMUs, choose between:
  - **Madgwick AHRS**: Gradient descent algorithm with low computational cost and magnetic distortion compensation.
  - **Mahony AHRS**: Proportional-integral (PI) observer with gyro drift integration on SO(3).
  - **Kalman AHRS**: 2-state per-axis linear Kalman filter estimating attitude and gyro zero-rate bias dynamically.
  - **Complementary Filter**: Classic frequency-splitting filter blending low-pass accelerometer tilt with high-pass integrated gyroscope rates.
* **🤝 Unified Front-End API**:
  Identical API across every IMU sensor: `getQuaternion()`, `getEuler()`, `getAccel()`, `getGyro()`, `getMag()`, `getLinearAccel()`, `getGravity()`.
* **🔀 Multi-IMU Orchestration**:
  Completely object-oriented without global static variables. Run multiple IMU instances concurrently across different I2C addresses, multiple I2C buses (`Wire`, `Wire1`), or multiple SPI chip select lines.
* **🛡️ Built-in Automated Calibration**:
  Instant stationary gyro bias estimation (`calibrateGyro()`) and accelerometer leveling (`calibrateAccel()`).

---

## 📊 Supported Sensor Matrix

| Manufacturer | Sensor Model | Axes | Bus Interfaces | Inbuilt Fusion Support | Typical Address / ID |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **Bosch** | **BNO055** | 9-DoF | I2C | **Yes** (Cortex-M0 BSX) | `0x28`, `0x29` (`0xA0`) |
| **Bosch** | **BNO080 / BNO085** | 9-DoF | I2C / SPI | **Yes** (SH-2 Hub) | `0x4A`, `0x4B` |
| **Bosch** | **BMI160 / BMI270** | 6-DoF | I2C / SPI | Software (Madgwick/Mahony) | `0x68`, `0x69` (`0xD1`/`0x24`) |
| **InvenSense** | **MPU-6050** | 6-DoF | I2C | DMP / Software | `0x68`, `0x69` (`0x68`) |
| **InvenSense** | **MPU-6500** | 6-DoF | I2C / SPI | DMP / Software | `0x68`, `0x69` (`0x70`) |
| **InvenSense** | **MPU-9250** | 9-DoF | I2C / SPI | DMP / Software (AK8963) | `0x68`, `0x69` (`0x71`) |
| **InvenSense** | **ICM-20948** | 9-DoF | I2C / SPI | DMP / Software (AK09916) | `0x68`, `0x69` (`0xEA`) |
| **InvenSense** | **ICM-42688** | 6-DoF | I2C / SPI | Software (Madgwick/Mahony) | `0x68`, `0x69` (`0x47`) |
| **STMicro** | **LSM6DSOX / LSM6DS3**| 6-DoF | I2C / SPI | Software (Madgwick/Mahony) | `0x6A`, `0x6B` (`0x6C`/`0x69`) |
| **STMicro** | **LSM9DS1** | 9-DoF | I2C / SPI | Software (Madgwick/Mahony) | `0x6A`/`0x1E` (`0x68`) |

---

## 📦 Installation

### Arduino IDE
1. Download or clone this repository:
   ```bash
   git clone https://github.com/thecorecomet/COMET_IMU.git
   ```
2. In Arduino IDE, navigate to **Sketch** -> **Include Library** -> **Add .ZIP Library...** and select the folder, OR copy the folder into your `Arduino/libraries/` directory.
3. Restart Arduino IDE. Examples will appear under **File** -> **Examples** -> **COMET_IMU**.

### PlatformIO
Add the repository or local path to your `platformio.ini`:
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =
    https://github.com/thecorecomet/COMET_IMU.git
```

---

## ⚡ Quick Start Examples

### 1. Auto-Detect Any Connected IMU (1-Line Initialization)
```cpp
#include <Wire.h>
#include <COMET_IMU.h>

COMET_IMU imu;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Automatically detects the connected IMU, starts fusion, and configures registers
    if (!imu.autoDetect(&Wire)) {
        Serial.println("No supported IMU found!");
        while (1);
    }

    Serial.print("Detected: ");
    Serial.println(imu.getSensorName());
    
    // Calibrate stationary gyro bias
    imu.calibrateGyro();
}

void loop() {
    if (imu.update()) {
        EulerAngles e = imu.getEuler();
        Serial.printf("Roll: %.2f | Pitch: %.2f | Yaw: %.2f\n", e.roll, e.pitch, e.yaw);
    }
    delay(20);
}
```

---

### 2. Multi-IMU Orchestration
```cpp
#include <Wire.h>
#include <COMET_IMU.h>

COMET_IMU imu1; // Primary IMU (e.g. MPU6050 at 0x68)
COMET_IMU imu2; // Secondary IMU (e.g. BNO055 at 0x28 or MPU6050 at 0x69)

void setup() {
    Serial.begin(115200);
    Wire.begin();

    imu1.beginI2C(IMU_MPU6050, 0x68);
    imu2.beginI2C(IMU_BNO055, 0x28);
}

void loop() {
    imu1.update();
    imu2.update();

    Serial.print("IMU1 Pitch: "); Serial.print(imu1.getEuler().pitch);
    Serial.print(" | IMU2 Pitch: "); Serial.println(imu2.getEuler().pitch);
    delay(20);
}
```

---

### 3. High-Speed SPI Auto-Detection
```cpp
#include <SPI.h>
#include <COMET_IMU.h>

#define CS_PIN 5

COMET_IMU imu;

void setup() {
    Serial.begin(115200);
    SPI.begin();

    // Auto-detect IMU connected to SPI with CS Pin 5
    if (!imu.autoDetectSPI(CS_PIN, &SPI)) {
        Serial.println("SPI IMU auto-detection failed!");
        while (1);
    }

    Serial.print("SPI IMU Active: ");
    Serial.println(imu.getSensorName());
}

void loop() {
    if (imu.update()) {
        Vector3 a = imu.getLinearAccel();
        Serial.printf("Dynamic Acceleration X: %.2f, Y: %.2f, Z: %.2f m/s2\n", a.x, a.y, a.z);
    }
    delay(10);
}
```

---

## 🛠️ Wiring Guide

### I2C Connection
| IMU Pin | ESP32 | STM32 (Nucleo) | Arduino Uno / Nano |
| :---: | :---: | :---: | :---: |
| **VCC** | 3.3V or 5V (Check board regulator) | 3.3V | 5V / 3.3V |
| **GND** | GND | GND | GND |
| **SDA** | GPIO 21 (or custom) | PB9 / D14 | A4 / SDA |
| **SCL** | GPIO 22 (or custom) | PB8 / D15 | A5 / SCL |

### SPI Connection
| IMU Pin | ESP32 | STM32 | Arduino Uno / Nano |
| :---: | :---: | :---: | :---: |
| **SCK / SCL** | GPIO 18 | PA5 / D13 | Pin 13 |
| **SDO / MISO**| GPIO 19 | PA6 / D12 | Pin 12 |
| **SDI / MOSI**| GPIO 23 | PA7 / D11 | Pin 11 |
| **CS** | User Selected (e.g. GPIO 5) | User Selected (e.g. D10) | User Selected (e.g. Pin 10)|

---

## 📖 API Reference

### Initialization & Front-End Setup
```cpp
bool autoDetect(TwoWire* wire = &Wire, const IMUConfig& config = IMUConfig());
bool autoDetectSPI(uint8_t csPin, SPIClass* spi = &SPI, const IMUConfig& config = IMUConfig());
bool begin(IMUType type, TwoWire* wire = &Wire, const IMUConfig& config = IMUConfig());
bool beginI2C(IMUType type, uint8_t address, TwoWire* wire = &Wire, const IMUConfig& config = IMUConfig());
bool beginSPI(IMUType type, uint8_t csPin, SPIClass* spi = &SPI, const IMUConfig& config = IMUConfig());
```

### Fusion Modes
```cpp
void setFusionMode(FusionMode mode);
// Modes:
// FUSION_AUTO          - Automatically offloads to hardware fusion if available, else Madgwick
// FUSION_HARDWARE      - Enforces on-chip hardware fusion (BNO055, BNO08x, DMP)
// FUSION_MADGWICK      - High-performance Madgwick 6-DoF / 9-DoF gradient-descent filter
// FUSION_MAHONY        - High-performance Mahony PI filter on SO(3)
// FUSION_KALMAN        - 2-state per-axis linear Kalman filter with dynamic gyro bias tracking
// FUSION_COMPLEMENTARY - Classic complementary filter blending low-pass accel with gyro rate
// FUSION_OFF           - Raw data only

// Custom fusion injection:
void setCustomFusion(SensorFusion* fusion, bool takeOwnership = false);
```

### Motion & Orientation Data Access
```cpp
bool update();                     // Refresh data and calculate fusion
Quaternion getQuaternion() const;  // Unit quaternion (w, x, y, z)
EulerAngles getEuler() const;      // Roll, Pitch, Yaw in degrees
Vector3 getAccel() const;          // Raw acceleration in m/s^2
Vector3 getGyro() const;           // Angular velocity in deg/s
Vector3 getMag() const;            // Magnetic flux in uT (9-DoF sensors)
Vector3 getLinearAccel() const;    // Dynamic acceleration (gravity vector subtracted)
Vector3 getGravity() const;        // Estimated unit gravity vector in sensor frame
float getTemperature() const;      // Sensor die temperature in Celsius
```

### Calibration
```cpp
void calibrateGyro(uint16_t samples = 300, uint16_t delayMs = 4);
void calibrateAccel(uint16_t samples = 300, uint16_t delayMs = 4);
```

---

## 📄 License
This library is released under the **MIT License**. Created by **Bhushan Patil**.
Contributions and pull requests are warmly welcomed at [https://github.com/thecorecomet/COMET_IMU.git](https://github.com/thecorecomet/COMET_IMU.git).
