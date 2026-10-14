/**
 * COMET_IMU - 01_AutoDetect_Any_IMU
 * 
 * Demonstrates 1-line automatic detection of any connected IMU sensor on I2C.
 * Automatically identifies:
 *   - Bosch: BNO055, BNO080, BNO085, BMI160, BMI270
 *   - InvenSense: MPU-6050, MPU-6500, MPU-9250, ICM-20948, ICM-42688
 *   - ST: LSM6DSOX, LSM6DS3, LSM9DS1
 * 
 * Auto-activates on-chip hardware sensor fusion if supported by the chip,
 * or switches to high-speed 6-DoF/9-DoF Madgwick AHRS filter.
 * 
 * Author: Bhushan Patil <bhushanshampatil@gmail.com>
 * Repo: https://github.com/thecorecomet/COMET_IMU.git
 */

#include <Wire.h>
#include <COMET_IMU.h>

COMET_IMU imu;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000); // Wait for serial port

    Serial.println();
    Serial.println("=========================================");
    Serial.println("    COMET_IMU: Auto-Detection Demo       ");
    Serial.println("=========================================");

    Wire.begin();
    Wire.setClock(400000); // 400kHz Fast Mode I2C

    Serial.println("Scanning I2C bus for connected IMU...");

    // 1-Line Auto-Detection:
    if (!imu.autoDetect(&Wire)) {
        Serial.println("[ERROR] No supported IMU detected on I2C bus!");
        Serial.println("Please verify SDA, SCL, VCC, and GND wiring.");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("[SUCCESS] Sensor Connected!");
    Serial.print("  Sensor Model:    "); Serial.println(imu.getSensorName());
    Serial.print("  Bus Interface:   "); Serial.println(imu.getBusType() == BUS_I2C ? "I2C" : "SPI");
    Serial.print("  Degrees of Free: "); Serial.println(imu.is9DOF() ? "9-DOF (Accel + Gyro + Mag)" : "6-DOF (Accel + Gyro)");
    Serial.print("  Hardware Fusion: "); Serial.println(imu.hasHardwareFusion() ? "ACTIVE (Zero CPU Load)" : "OFF (Madgwick AHRS active)");

    Serial.println("\nCalibrating Gyroscope (Keep sensor still)...");
    imu.calibrateGyro(300, 4);
    Serial.println("Calibration complete!\n");

    Serial.println("Streaming Orientation & Motion Data (Format: Roll, Pitch, Yaw in degrees):");
    Serial.println("-------------------------------------------------------------------------");
}

void loop() {
    if (imu.update()) {
        EulerAngles euler = imu.getEuler();
        Quaternion q = imu.getQuaternion();
        Vector3 linA = imu.getLinearAccel();
        float temp = imu.getTemperature();

        // Print orientation
        Serial.print("Roll: ");   Serial.print(euler.roll, 2);
        Serial.print(" | Pitch: "); Serial.print(euler.pitch, 2);
        Serial.print(" | Yaw: ");   Serial.print(euler.yaw, 2);

        // Print quaternion
        Serial.print(" | Quat: [");
        Serial.print(q.w, 3); Serial.print(", ");
        Serial.print(q.x, 3); Serial.print(", ");
        Serial.print(q.y, 3); Serial.print(", ");
        Serial.print(q.z, 3); Serial.print("]");

        // Print linear acceleration & temperature
        Serial.print(" | LinAccZ: "); Serial.print(linA.z, 2);
        Serial.print(" m/s2 | Temp: "); Serial.print(temp, 1);
        Serial.println(" C");
    }

    delay(20); // ~50Hz stream rate
}
