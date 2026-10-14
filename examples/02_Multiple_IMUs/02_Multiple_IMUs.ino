/**
 * COMET_IMU - 02_Multiple_IMUs
 * 
 * Demonstrates orchestrating multiple IMU sensors simultaneously.
 * Ideal for multi-joint robotics, dual-sensor fuselage tracking,
 * or comparative benchmarking of sensors.
 * 
 * In this example:
 *   - IMU 1 is connected via I2C at address 0x68 (e.g. MPU6050 with AD0=GND)
 *   - IMU 2 is connected via I2C at address 0x69 (e.g. MPU6050 with AD0=VCC)
 *     or address 0x28 (e.g. Bosch BNO055)
 * 
 * Author: Bhushan Patil <bhushanshampatil@gmail.com>
 * Repo: https://github.com/thecorecomet/COMET_IMU.git
 */

#include <Wire.h>
#include <COMET_IMU.h>

// Create independent instances for each IMU
COMET_IMU imu1;
COMET_IMU imu2;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println();
    Serial.println("=========================================");
    Serial.println("    COMET_IMU: Dual IMU Orchestration   ");
    Serial.println("=========================================");

    Wire.begin();
    Wire.setClock(400000);

    // Initialize IMU 1 on I2C address 0x68
    Serial.print("Initializing IMU 1 (0x68)... ");
    if (imu1.beginI2C(IMU_MPU6050, 0x68, &Wire)) {
        Serial.print("OK! Sensor: "); Serial.println(imu1.getSensorName());
    } else {
        Serial.println("Failed! Trying auto-detect on 0x68...");
        imu1.autoDetect(&Wire);
    }

    // Initialize IMU 2 on I2C address 0x69 (or 0x28 for BNO055)
    Serial.print("Initializing IMU 2 (0x69 or 0x28)... ");
    if (imu2.beginI2C(IMU_MPU6050, 0x69, &Wire)) {
        Serial.print("OK! Sensor: "); Serial.println(imu2.getSensorName());
    } else if (imu2.beginI2C(IMU_BNO055, 0x28, &Wire)) {
        Serial.print("OK! Sensor: "); Serial.println(imu2.getSensorName());
    } else {
        Serial.println("Failed! Check secondary IMU address.");
    }

    Serial.println("\nCalibrating sensors in stationary position...");
    if (imu1.isInitialized()) imu1.calibrateGyro(200, 3);
    if (imu2.isInitialized()) imu2.calibrateGyro(200, 3);
    Serial.println("Calibration complete!\n");
}

void loop() {
    // Update both IMU instances
    if (imu1.isInitialized()) imu1.update();
    if (imu2.isInitialized()) imu2.update();

    // Fetch orientation from both
    EulerAngles e1 = imu1.getEuler();
    EulerAngles e2 = imu2.getEuler();

    // Print side-by-side pitch and roll comparison
    Serial.print("[IMU 1] P: "); Serial.print(e1.pitch, 1);
    Serial.print(" R: ");       Serial.print(e1.roll, 1);
    Serial.print(" Y: ");       Serial.print(e1.yaw, 1);

    Serial.print("  ||  [IMU 2] P: "); Serial.print(e2.pitch, 1);
    Serial.print(" R: ");             Serial.print(e2.roll, 1);
    Serial.print(" Y: ");             Serial.print(e2.yaw, 1);

    Serial.println();
    delay(25);
}
