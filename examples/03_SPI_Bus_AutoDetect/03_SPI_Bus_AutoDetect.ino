/**
 * COMET_IMU - 03_SPI_Bus_AutoDetect
 * 
 * Demonstrates high-speed SPI auto-detection and data acquisition.
 * Ideal for ICM-20948, MPU-9250, ICM-42688, or LSM6DSOX where high ODR
 * (1kHz - 8kHz) and low bus latency are required.
 * 
 * Hardware Wiring (Standard Arduino/ESP32 SPI):
 *   - SCK  -> SCK (Pin 18 on ESP32 / Pin 13 on Arduino Uno)
 *   - MISO / SDO -> MISO (Pin 19 on ESP32 / Pin 12 on Uno)
 *   - MOSI / SDI -> MOSI (Pin 23 on ESP32 / Pin 11 on Uno)
 *   - CS   -> Configurable Chip Select pin (e.g. Pin 5)
 * 
 * Author: Bhushan Patil <bhushanshampatil@gmail.com>
 * Repo: https://github.com/thecorecomet/COMET_IMU.git
 */

#include <SPI.h>
#include <COMET_IMU.h>

#define IMU_CS_PIN 5 // Chip Select Pin

COMET_IMU imu;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println();
    Serial.println("=========================================");
    Serial.println("    COMET_IMU: High-Speed SPI Demo       ");
    Serial.println("=========================================");

    SPI.begin();

    Serial.print("Probing SPI bus on CS Pin ");
    Serial.println(IMU_CS_PIN);

    // Auto-detect SPI IMU on the given CS pin:
    if (!imu.autoDetectSPI(IMU_CS_PIN, &SPI)) {
        Serial.println("[ERROR] Failed to auto-detect IMU over SPI!");
        Serial.println("Check MOSI, MISO, SCK, CS wiring and power supply.");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("[SUCCESS] Sensor Connected via SPI!");
    Serial.print("  Sensor Model: "); Serial.println(imu.getSensorName());
    Serial.print("  Interface:    SPI (Hardware Bus)\n");

    imu.calibrateGyro(250, 4);
    Serial.println("Gyro calibrated successfully!\n");
}

void loop() {
    if (imu.update()) {
        EulerAngles e = imu.getEuler();
        Vector3 a = imu.getAccel();
        Vector3 g = imu.getGyro();

        Serial.print("Roll: "); Serial.print(e.roll, 2);
        Serial.print(" | Pitch: "); Serial.print(e.pitch, 2);
        Serial.print(" | Yaw: "); Serial.print(e.yaw, 2);
        Serial.print(" || AccelZ: "); Serial.print(a.z, 2);
        Serial.print(" m/s2 | GyroZ: "); Serial.print(g.z, 2);
        Serial.println(" deg/s");
    }

    delay(10); // 100Hz loop
}
