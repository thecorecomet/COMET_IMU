/**
 * COMET_IMU - 04_Hardware_vs_Software_Fusion
 * 
 * Demonstrates the power of Inbuilt Hardware Sensor Fusion vs Software Fusion.
 * 
 * Features demonstrated:
 *   1. Hardware Fusion: Offloads orientation computation entirely to on-chip MCU
 *      (e.g. Bosch BNO055 Cortex-M0 or BNO085 SH-2 hub), using ZERO host MCU cycles.
 *   2. Software Fusion: High-speed Madgwick & Mahony AHRS filters with variable gains.
 *   3. Microsecond timing benchmark comparing CPU execution overhead!
 * 
 * Author: Bhushan Patil <bhushanshampatil@gmail.com>
 * Repo: https://github.com/thecorecomet/COMET_IMU.git
 */

#include <Wire.h>
#include <COMET_IMU.h>

COMET_IMU imu;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println();
    Serial.println("====================================================");
    Serial.println("  COMET_IMU: Hardware vs Software Fusion Benchmark  ");
    Serial.println("====================================================");

    Wire.begin();
    Wire.setClock(400000);

    if (!imu.autoDetect(&Wire)) {
        Serial.println("[ERROR] No IMU found! Please check connections.");
        while(1) delay(1000);
    }

    Serial.print("Detected Sensor: "); Serial.println(imu.getSensorName());
    Serial.print("Has Inbuilt Hardware Fusion: ");
    Serial.println(imu.hasHardwareFusion() ? "YES" : "NO");

    // Benchmark 1: Hardware Fusion (if supported) or Auto
    Serial.println("\nTesting FUSION_AUTO mode...");
    imu.setFusionMode(FUSION_AUTO);
    benchmarkFilter("FUSION_AUTO");

    // Benchmark 2: Madgwick Filter
    Serial.println("Testing FUSION_MADGWICK mode...");
    imu.setFusionMode(FUSION_MADGWICK);
    benchmarkFilter("Madgwick AHRS");

    // Benchmark 3: Mahony Filter
    Serial.println("Testing FUSION_MAHONY mode...");
    imu.setFusionMode(FUSION_MAHONY);
    benchmarkFilter("Mahony AHRS");

    Serial.println("\nStarting continuous streaming in FUSION_AUTO mode:");
    imu.setFusionMode(FUSION_AUTO);
}

void benchmarkFilter(const char* name) {
    uint32_t start = micros();
    const int iterations = 100;
    for (int i = 0; i < iterations; i++) {
        imu.update();
    }
    uint32_t total = micros() - start;
    float avgUs = (float)total / (float)iterations;

    Serial.print("  -> "); Serial.print(name);
    Serial.print(" Average update time: ");
    Serial.print(avgUs, 2);
    Serial.println(" microseconds per frame");
}

void loop() {
    if (imu.update()) {
        EulerAngles e = imu.getEuler();
        Serial.print("Roll: "); Serial.print(e.roll, 1);
        Serial.print(" | Pitch: "); Serial.print(e.pitch, 1);
        Serial.print(" | Yaw: "); Serial.println(e.yaw, 1);
    }
    delay(20);
}
