#include "DriverICM42688.h"

DriverICM42688::DriverICM42688(BusInterface* bus)
    : BaseIMUDriver(bus), 
      _accelScaleFactor(9.80665f / 8192.0f), 
      _gyroScaleFactor(1.0f / 32.8f) {}

bool DriverICM42688::begin(const IMUConfig& config) {
    if (!_bus) return false;
    _bus->begin();

    // Soft reset (DEVICE_CONFIG reg 0x11, bit 0)
    _bus->writeByte(0x11, 0x01);
    delay(50);

    // Turn on Gyro & Accel in Low-Noise mode (PWR_MGMT0 reg 0x4E = 0x0F)
    _bus->writeByte(0x4E, 0x0F);
    delay(45);

    // Gyro Config 0 (reg 0x4F): bits 7:5 FSR, bits 3:0 ODR
    uint8_t gyroFsrBits = 0x20; // Default 1000 dps
    if (config.gyroFsrDps <= 250) {
        gyroFsrBits = 0x60;
        _gyroScaleFactor = 1.0f / 131.0f;
    } else if (config.gyroFsrDps <= 500) {
        gyroFsrBits = 0x40;
        _gyroScaleFactor = 1.0f / 65.5f;
    } else if (config.gyroFsrDps <= 1000) {
        gyroFsrBits = 0x20;
        _gyroScaleFactor = 1.0f / 32.8f;
    } else {
        gyroFsrBits = 0x00; // 2000 dps
        _gyroScaleFactor = 1.0f / 16.4f;
    }
    _bus->writeByte(0x4F, gyroFsrBits | 0x07); // 200Hz ODR

    // Accel Config 0 (reg 0x50): bits 7:5 FSR, bits 3:0 ODR
    uint8_t accelFsrBits = 0x40; // Default 4G
    if (config.accelFsrG <= 2) {
        accelFsrBits = 0x60;
        _accelScaleFactor = 9.80665f / 16384.0f;
    } else if (config.accelFsrG <= 4) {
        accelFsrBits = 0x40;
        _accelScaleFactor = 9.80665f / 8192.0f;
    } else if (config.accelFsrG <= 8) {
        accelFsrBits = 0x20;
        _accelScaleFactor = 9.80665f / 4096.0f;
    } else {
        accelFsrBits = 0x00; // 16G
        _accelScaleFactor = 9.80665f / 2048.0f;
    }
    _bus->writeByte(0x50, accelFsrBits | 0x07); // 200Hz ODR

    return true;
}

bool DriverICM42688::readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) {
    if (!_bus) return false;

    // Read 14 bytes starting at TEMP_DATA1 (0x1D)
    uint8_t buffer[14];
    if (!_bus->readBytes(0x1D, buffer, 14)) {
        return false;
    }

    int16_t rawTemp = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t rawAx   = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t rawAy   = (int16_t)((buffer[4] << 8) | buffer[5]);
    int16_t rawAz   = (int16_t)((buffer[6] << 8) | buffer[7]);
    int16_t rawGx   = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t rawGy   = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t rawGz   = (int16_t)((buffer[12] << 8) | buffer[13]);

    accel.x = (float)rawAx * _accelScaleFactor;
    accel.y = (float)rawAy * _accelScaleFactor;
    accel.z = (float)rawAz * _accelScaleFactor;

    gyro.x = (float)rawGx * _gyroScaleFactor;
    gyro.y = (float)rawGy * _gyroScaleFactor;
    gyro.z = (float)rawGz * _gyroScaleFactor;

    temp = ((float)rawTemp / 132.48f) + 25.0f;

    mag.x = 0; mag.y = 0; mag.z = 0;

    applyCalibration(accel, gyro, mag);
    return true;
}
