#include "DriverLSM6DSOX.h"

DriverLSM6DSOX::DriverLSM6DSOX(BusInterface* bus)
    : BaseIMUDriver(bus), 
      _accelScaleFactor(9.80665f / 8192.0f), 
      _gyroScaleFactor(1.0f / 28.57f), 
      _isLSM6DS3(false) {}

bool DriverLSM6DSOX::begin(const IMUConfig& config) {
    if (!_bus) return false;
    _bus->begin();

    // Check WHO_AM_I (0x0F)
    uint8_t id = _bus->readByte(0x0F);
    if (id == 0x69) {
        _isLSM6DS3 = true;
    } else {
        _isLSM6DS3 = false;
    }

    // Soft reset (CTRL3_C reg 0x12, bit 0 = SW_RESET = 0x01)
    _bus->writeByte(0x12, 0x01);
    delay(50);

    // Set BDU (Block Data Update bit 6) and IF_INC (Auto increment bit 2) = 0x44
    _bus->writeByte(0x12, 0x44);

    // Accel Config (CTRL1_XL reg 0x10): ODR + FSR
    uint8_t accelFsrBits = 0x08; // Default +/- 4G (0.122 mg/LSB = 8192 LSB/g)
    if (config.accelFsrG <= 2) {
        accelFsrBits = 0x00;
        _accelScaleFactor = (9.80665f * 0.061f) / 1000.0f; // 0.061 mg/LSB
    } else if (config.accelFsrG <= 4) {
        accelFsrBits = 0x08;
        _accelScaleFactor = (9.80665f * 0.122f) / 1000.0f; // 0.122 mg/LSB
    } else if (config.accelFsrG <= 8) {
        accelFsrBits = 0x0C;
        _accelScaleFactor = (9.80665f * 0.244f) / 1000.0f; // 0.244 mg/LSB
    } else {
        accelFsrBits = 0x04;
        _accelScaleFactor = (9.80665f * 0.488f) / 1000.0f; // 0.488 mg/LSB
    }
    // Set 208Hz ODR (0x50) + FSR bits
    _bus->writeByte(0x10, 0x50 | accelFsrBits);

    // Gyro Config (CTRL2_G reg 0x11): ODR + FSR
    uint8_t gyroFsrBits = 0x08; // Default 1000 dps (35 mdps/LSB)
    if (config.gyroFsrDps <= 250) {
        gyroFsrBits = 0x00;
        _gyroScaleFactor = 8.75f / 1000.0f;
    } else if (config.gyroFsrDps <= 500) {
        gyroFsrBits = 0x04;
        _gyroScaleFactor = 17.50f / 1000.0f;
    } else if (config.gyroFsrDps <= 1000) {
        gyroFsrBits = 0x08;
        _gyroScaleFactor = 35.0f / 1000.0f;
    } else {
        gyroFsrBits = 0x0C; // 2000 dps (70 mdps/LSB)
        _gyroScaleFactor = 70.0f / 1000.0f;
    }
    // Set 208Hz ODR (0x50) + FSR bits
    _bus->writeByte(0x11, 0x50 | gyroFsrBits);

    return true;
}

bool DriverLSM6DSOX::readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) {
    if (!_bus) return false;

    // Read 14 bytes starting at OUT_TEMP_L (0x20): Temp (2), Gyro (6), Accel (6)
    uint8_t buffer[14];
    if (!_bus->readBytes(0x20, buffer, 14)) {
        return false;
    }

    int16_t rawTemp = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t rawGx   = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t rawGy   = (int16_t)((buffer[5] << 8) | buffer[4]);
    int16_t rawGz   = (int16_t)((buffer[7] << 8) | buffer[6]);
    int16_t rawAx   = (int16_t)((buffer[9] << 8) | buffer[8]);
    int16_t rawAy   = (int16_t)((buffer[11] << 8) | buffer[10]);
    int16_t rawAz   = (int16_t)((buffer[13] << 8) | buffer[12]);

    accel.x = (float)rawAx * _accelScaleFactor;
    accel.y = (float)rawAy * _accelScaleFactor;
    accel.z = (float)rawAz * _accelScaleFactor;

    gyro.x = (float)rawGx * _gyroScaleFactor;
    gyro.y = (float)rawGy * _gyroScaleFactor;
    gyro.z = (float)rawGz * _gyroScaleFactor;

    temp = ((float)rawTemp / 256.0f) + 25.0f;

    mag.x = 0; mag.y = 0; mag.z = 0;

    applyCalibration(accel, gyro, mag);
    return true;
}
