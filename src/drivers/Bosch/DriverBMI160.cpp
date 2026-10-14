#include "DriverBMI160.h"

DriverBMI160::DriverBMI160(BusInterface* bus)
    : BaseIMUDriver(bus), 
      _accelScaleFactor(9.80665f / 8192.0f), 
      _gyroScaleFactor(1.0f / 32.8f), 
      _isBMI270(false) {}

bool DriverBMI160::begin(const IMUConfig& config) {
    if (!_bus) return false;
    _bus->begin();

    // Check Chip ID
    uint8_t chipId = _bus->readByte(0x00);
    if (chipId == 0x24) {
        _isBMI270 = true;
    } else if (chipId == 0xD1) {
        _isBMI270 = false;
    }

    // Reset device
    _bus->writeByte(0x7E, 0xB6);
    delay(50);

    // Set Accel to Normal Mode
    _bus->writeByte(0x7E, 0x11);
    delay(10);

    // Set Gyro to Normal Mode
    _bus->writeByte(0x7E, 0x15);
    delay(60); // Gyro startup time is ~55ms

    // Accel range config (reg 0x41)
    uint8_t accRange = 0x05; // Default 4G
    if (config.accelFsrG <= 2) {
        accRange = 0x03;
        _accelScaleFactor = 9.80665f / 16384.0f;
    } else if (config.accelFsrG <= 4) {
        accRange = 0x05;
        _accelScaleFactor = 9.80665f / 8192.0f;
    } else if (config.accelFsrG <= 8) {
        accRange = 0x08;
        _accelScaleFactor = 9.80665f / 4096.0f;
    } else {
        accRange = 0x0C;
        _accelScaleFactor = 9.80665f / 2048.0f;
    }
    _bus->writeByte(0x41, accRange);

    // Gyro range config (reg 0x43)
    uint8_t gyrRange = 0x01; // Default 1000 dps
    if (config.gyroFsrDps <= 250) {
        gyrRange = 0x03;
        _gyroScaleFactor = 1.0f / 131.0f;
    } else if (config.gyroFsrDps <= 500) {
        gyrRange = 0x02;
        _gyroScaleFactor = 1.0f / 65.5f;
    } else if (config.gyroFsrDps <= 1000) {
        gyrRange = 0x01;
        _gyroScaleFactor = 1.0f / 32.8f;
    } else {
        gyrRange = 0x00;
        _gyroScaleFactor = 1.0f / 16.4f;
    }
    _bus->writeByte(0x43, gyrRange);

    return true;
}

bool DriverBMI160::readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) {
    if (!_bus) return false;

    // Read 12 bytes starting at GYR_DATA_X (0x0C): 6 bytes Gyro, then 6 bytes Accel
    uint8_t buffer[12];
    if (!_bus->readBytes(0x0C, buffer, 12)) {
        return false;
    }

    int16_t rawGx = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t rawGy = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t rawGz = (int16_t)((buffer[5] << 8) | buffer[4]);

    int16_t rawAx = (int16_t)((buffer[7] << 8) | buffer[6]);
    int16_t rawAy = (int16_t)((buffer[9] << 8) | buffer[8]);
    int16_t rawAz = (int16_t)((buffer[11] << 8) | buffer[10]);

    accel.x = (float)rawAx * _accelScaleFactor;
    accel.y = (float)rawAy * _accelScaleFactor;
    accel.z = (float)rawAz * _accelScaleFactor;

    gyro.x = (float)rawGx * _gyroScaleFactor;
    gyro.y = (float)rawGy * _gyroScaleFactor;
    gyro.z = (float)rawGz * _gyroScaleFactor;

    mag.x = 0; mag.y = 0; mag.z = 0;

    // Read temperature (0x20)
    uint8_t tBuf[2];
    if (_bus->readBytes(0x20, tBuf, 2)) {
        int16_t rawT = (int16_t)((tBuf[1] << 8) | tBuf[0]);
        temp = 23.0f + ((float)rawT / 512.0f);
    } else {
        temp = 25.0f;
    }

    applyCalibration(accel, gyro, mag);
    return true;
}
