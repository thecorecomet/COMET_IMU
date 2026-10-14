/**
 * COMET_IMU - 05_ESP32_STM32_Advanced
 * 
 * Demonstrates advanced configuration for ESP32 and STM32 microcontrollers:
 *   - Custom I2C pin mapping (e.g. GPIO 21/22 or custom ESP32-S3 pins)
 *   - High-speed 400kHz I2C clock configuration
 *   - Gravity isolation & Linear Acceleration monitoring
 *   - Dynamic Impact / Free-Fall / Shake detection
 * 
 * Author: Bhushan Patil <bhushanshampatil@gmail.com>
 * Repo: https://github.com/thecorecomet/COMET_IMU.git
 */

#include <Wire.h>
#include <COMET_IMU.h>

#if defined(ESP32)
  #define CUSTOM_SDA 21
  #define CUSTOM_SCL 22
#elif defined(ARDUINO_ARCH_STM32)
  #define CUSTOM_SDA PB9
  #define CUSTOM_SCL PB8
#else
  // Standard default Arduino pins (A4/A5 on Uno)
  #define CUSTOM_SDA -1
  #define CUSTOM_SCL -1
#endif

COMET_IMU imu;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println();
    Serial.println("====================================================");
    Serial.println("     COMET_IMU: ESP32 & STM32 Advanced Demo         ");
    Serial.println("====================================================");

    #if defined(ESP32)
      Serial.printf("Configuring ESP32 I2C on SDA: %d, SCL: %d\n", CUSTOM_SDA, CUSTOM_SCL);
      Wire.begin(CUSTOM_SDA, CUSTOM_SCL);
    #else
      Wire.begin();
    #endif

    Wire.setClock(400000); // 400kHz Fast Mode

    IMUConfig config;
    config.sampleRateHz = 200; // 200Hz high rate
    config.accelFsrG = 8;      // 8G range for high dynamics
    config.gyroFsrDps = 2000;  // 2000 dps
    config.enableMag = true;
    config.fusionMode = FUSION_AUTO;

    if (!imu.autoDetect(&Wire, config)) {
        Serial.println("[ERROR] Failed to find IMU! Please check wiring.");
        while(1) delay(1000);
    }

    Serial.println("[SUCCESS] Initialized with Advanced Config!");
    Serial.print("  Sensor: "); Serial.println(imu.getSensorName());
    Serial.print("  Fusion: "); Serial.println(imu.hasHardwareFusion() ? "Hardware (Inbuilt)" : "Software (Madgwick)");

    imu.calibrateGyro(300, 3);
    Serial.println("Stationary calibration done!\n");
}

void loop() {
    static uint32_t lastPrint = 0;

    if (imu.update()) {
        Vector3 linA = imu.getLinearAccel();
        float totalLinAccel = linA.length(); // Total magnitude of dynamic acceleration (m/s^2)

        // Free-fall detection (near 0 total acceleration when falling)
        Vector3 rawA = imu.getAccel();
        if (rawA.length() < 2.0f) {
            Serial.println(">>> [EVENT] FREE-FALL DETECTED! <<<");
        }

        // High Impact / Shock detection
        if (totalLinAccel > 25.0f) {
            Serial.print(">>> [EVENT] IMPACT DETECTED: ");
            Serial.print(totalLinAccel);
            Serial.println(" m/s^2 <<<");
        }

        // Print regular telemetry every 50ms (20Hz)
        if (millis() - lastPrint >= 50) {
            lastPrint = millis();
            EulerAngles e = imu.getEuler();
            Serial.printf("R: %6.1f | P: %6.1f | Y: %6.1f || LinAcc: %5.2f m/s2\n", 
                          e.roll, e.pitch, e.yaw, totalLinAccel);
        }
    }
}
