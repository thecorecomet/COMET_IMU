#include "DriverMPU6050.h"

DriverMPU6050::DriverMPU6050(BusInterface* bus) 
    : BaseIMUDriver(bus), _accelScaleFactor(9.80665f / 8192.0f), _gyroScaleFactor(1.0f / 32.8f) {}

bool DriverMPU6050::begin(const IMUConfig& config) {
    if (!_bus) return false;
    _bus->begin();

    // Reset device
    _bus->writeByte(0x6B, 0x80);
    delay(50);

    // Wake up chip and select PLL with X-axis gyroscope reference
    _bus->writeByte(0x6B, 0x01);
    delay(10);

    // Enable Accel & Gyro
    _bus->writeByte(0x6C, 0x00);

    // Set sample rate divider
    _bus->writeByte(0x19, 0x04); // 200Hz if 1kHz gyro rate

    // Set DLPF (Digital Low Pass Filter) to ~44Hz bandwidth
    _bus->writeByte(0x1A, 0x03);

    // Configure Gyro Full Scale Range
    uint8_t gyroConfigVal = 0x10; // Default 1000 dps
    if (config.gyroFsrDps <= 250) {
        gyroConfigVal = 0x00;
        _gyroScaleFactor = 1.0f / 131.0f;
    } else if (config.gyroFsrDps <= 500) {
        gyroConfigVal = 0x08;
        _gyroScaleFactor = 1.0f / 65.5f;
    } else if (config.gyroFsrDps <= 1000) {
        gyroConfigVal = 0x10;
        _gyroScaleFactor = 1.0f / 32.8f;
    } else {
        gyroConfigVal = 0x18;
        _gyroScaleFactor = 1.0f / 16.4f;
    }
    _bus->writeByte(0x1B, gyroConfigVal);

    // Configure Accel Full Scale Range
    uint8_t accelConfigVal = 0x08; // Default +/- 4G
    if (config.accelFsrG <= 2) {
        accelConfigVal = 0x00;
        _accelScaleFactor = 9.80665f / 16384.0f;
    } else if (config.accelFsrG <= 4) {
        accelConfigVal = 0x08;
        _accelScaleFactor = 9.80665f / 8192.0f;
    } else if (config.accelFsrG <= 8) {
        accelConfigVal = 0x10;
        _accelScaleFactor = 9.80665f / 4096.0f;
    } else {
        accelConfigVal = 0x18;
        _accelScaleFactor = 9.80665f / 2048.0f;
    }
    _bus->writeByte(0x1C, accelConfigVal);

    return true;
}

bool DriverMPU6050::readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) {
    if (!_bus) return false;

    uint8_t buffer[14];
    if (!_bus->readBytes(0x3B, buffer, 14)) {
        return false;
    }

    int16_t rawAx = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t rawAy = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t rawAz = (int16_t)((buffer[4] << 8) | buffer[5]);

    int16_t rawTemp = (int16_t)((buffer[6] << 8) | buffer[7]);

    int16_t rawGx = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t rawGy = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t rawGz = (int16_t)((buffer[12] << 8) | buffer[13]);

    accel.x = (float)rawAx * _accelScaleFactor;
    accel.y = (float)rawAy * _accelScaleFactor;
    accel.z = (float)rawAz * _accelScaleFactor;

    gyro.x = (float)rawGx * _gyroScaleFactor;
    gyro.y = (float)rawGy * _gyroScaleFactor;
    gyro.z = (float)rawGz * _gyroScaleFactor;

    temp = ((float)rawTemp) / 340.0f + 36.53f;

    // No magnetometer on MPU-6050
    mag.x = 0.0f;
    mag.y = 0.0f;
    mag.z = 0.0f;

    applyCalibration(accel, gyro, mag);
    return true;
}
