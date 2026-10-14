#include "DriverLSM9DS1.h"
#include "../../bus/I2CBus.h"

DriverLSM9DS1::DriverLSM9DS1(BusInterface* bus)
    : BaseIMUDriver(bus), 
      _accelScaleFactor(9.80665f / 8192.0f), 
      _gyroScaleFactor(1.0f / 28.57f), 
      _magScaleFactor(0.14f), 
      _magI2CAddr(0x1E), 
      _hasMag(false) {}

bool DriverLSM9DS1::initMag() {
    if (_bus->getType() == BUS_I2C) {
        I2CBus* i2c = static_cast<I2CBus*>(_bus);
        TwoWire* wire = i2c->getWire();

        // Check if Mag responds at 0x1E or 0x1C
        wire->beginTransmission(_magI2CAddr);
        if (wire->endTransmission() != 0) {
            _magI2CAddr = (_magI2CAddr == 0x1E) ? 0x1C : 0x1E;
            wire->beginTransmission(_magI2CAddr);
            if (wire->endTransmission() != 0) {
                return false;
            }
        }

        // Verify WHO_AM_I_M (0x0F) == 0x3D
        wire->beginTransmission(_magI2CAddr);
        wire->write(0x0F);
        if (wire->endTransmission() != 0) return false;
        wire->requestFrom(_magI2CAddr, (uint8_t)1);
        if (!wire->available() || wire->read() != 0x3D) return false;

        // CTRL_REG1_M (0x20): Temp comp enable (bit 7) + High performance (bits 6:5 = 0x60) + 80Hz ODR (bits 4:2 = 0x1C) = 0xFC
        wire->beginTransmission(_magI2CAddr);
        wire->write(0x20);
        wire->write(0xFC);
        wire->endTransmission();

        // CTRL_REG2_M (0x21): +/- 4 Gauss full scale (bits 6:5 = 0x00) -> 0.14 mgauss/LSB = 0.014 uT/LSB
        wire->beginTransmission(_magI2CAddr);
        wire->write(0x21);
        wire->write(0x00);
        wire->endTransmission();
        _magScaleFactor = 0.014f;

        // CTRL_REG3_M (0x22): Continuous conversion mode (0x00)
        wire->beginTransmission(_magI2CAddr);
        wire->write(0x22);
        wire->write(0x00);
        wire->endTransmission();

        // CTRL_REG4_M (0x23): High performance Z-axis (bits 3:2 = 0x08)
        wire->beginTransmission(_magI2CAddr);
        wire->write(0x23);
        wire->write(0x08);
        wire->endTransmission();

        return true;
    }
    return false;
}

bool DriverLSM9DS1::readMag(Vector3& mag) {
    if (!_hasMag || _bus->getType() != BUS_I2C) {
        mag.x = 0; mag.y = 0; mag.z = 0;
        return false;
    }

    I2CBus* i2c = static_cast<I2CBus*>(_bus);
    TwoWire* wire = i2c->getWire();

    wire->beginTransmission(_magI2CAddr);
    wire->write(0x28 | 0x80); // Auto-increment read
    if (wire->endTransmission() != 0) return false;

    wire->requestFrom(_magI2CAddr, (uint8_t)6);
    if (wire->available() < 6) return false;

    int16_t rawMx = (int16_t)(wire->read() | (wire->read() << 8));
    int16_t rawMy = (int16_t)(wire->read() | (wire->read() << 8));
    int16_t rawMz = (int16_t)(wire->read() | (wire->read() << 8));

    mag.x = (float)rawMx * _magScaleFactor;
    mag.y = (float)rawMy * _magScaleFactor;
    mag.z = (float)rawMz * _magScaleFactor;

    return true;
}

bool DriverLSM9DS1::begin(const IMUConfig& config) {
    if (!_bus) return false;
    _bus->begin();

    // Reset Accel/Gyro: CTRL_REG8 (0x22) bit 0 = 0x01
    _bus->writeByte(0x22, 0x01);
    delay(50);

    // Gyro Config: CTRL_REG1_G (0x10): ODR 119Hz (0x60) + FSR
    uint8_t gyroFsrBits = 0x08; // Default 500 dps (17.5 mdps/LSB)
    if (config.gyroFsrDps <= 250) {
        gyroFsrBits = 0x00;
        _gyroScaleFactor = 8.75f / 1000.0f;
    } else if (config.gyroFsrDps <= 500) {
        gyroFsrBits = 0x08;
        _gyroScaleFactor = 17.5f / 1000.0f;
    } else {
        gyroFsrBits = 0x18; // 2000 dps (70 mdps/LSB)
        _gyroScaleFactor = 70.0f / 1000.0f;
    }
    _bus->writeByte(0x10, 0x60 | gyroFsrBits);

    // Accel Config: CTRL_REG6_XL (0x20): ODR 119Hz (0x60) + FSR
    uint8_t accelFsrBits = 0x10; // Default +/- 4G (0.122 mg/LSB)
    if (config.accelFsrG <= 2) {
        accelFsrBits = 0x00;
        _accelScaleFactor = (9.80665f * 0.061f) / 1000.0f;
    } else if (config.accelFsrG <= 4) {
        accelFsrBits = 0x10;
        _accelScaleFactor = (9.80665f * 0.122f) / 1000.0f;
    } else if (config.accelFsrG <= 8) {
        accelFsrBits = 0x18;
        _accelScaleFactor = (9.80665f * 0.244f) / 1000.0f;
    } else {
        accelFsrBits = 0x08; // 16G (0.732 mg/LSB)
        _accelScaleFactor = (9.80665f * 0.732f) / 1000.0f;
    }
    _bus->writeByte(0x20, 0x60 | accelFsrBits);

    // CTRL_REG8 (0x22): BDU (bit 6 = 0x40) + IF_ADD_INC (bit 2 = 0x04)
    _bus->writeByte(0x22, 0x44);

    if (config.enableMag) {
        _hasMag = initMag();
    }

    return true;
}

bool DriverLSM9DS1::readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) {
    if (!_bus) return false;

    // Read Gyro: 6 bytes from OUT_X_L_G (0x18)
    uint8_t gBuf[6];
    if (!_bus->readBytes(0x18, gBuf, 6)) return false;

    // Read Accel: 6 bytes from OUT_X_L_XL (0x28)
    uint8_t aBuf[6];
    if (!_bus->readBytes(0x28, aBuf, 6)) return false;

    int16_t rawGx = (int16_t)(gBuf[0] | (gBuf[1] << 8));
    int16_t rawGy = (int16_t)(gBuf[2] | (gBuf[3] << 8));
    int16_t rawGz = (int16_t)(gBuf[4] | (gBuf[5] << 8));

    int16_t rawAx = (int16_t)(aBuf[0] | (aBuf[1] << 8));
    int16_t rawAy = (int16_t)(aBuf[2] | (aBuf[3] << 8));
    int16_t rawAz = (int16_t)(aBuf[4] | (aBuf[5] << 8));

    accel.x = (float)rawAx * _accelScaleFactor;
    accel.y = (float)rawAy * _accelScaleFactor;
    accel.z = (float)rawAz * _accelScaleFactor;

    gyro.x = (float)rawGx * _gyroScaleFactor;
    gyro.y = (float)rawGy * _gyroScaleFactor;
    gyro.z = (float)rawGz * _gyroScaleFactor;

    temp = 25.0f;

    if (_hasMag) {
        readMag(mag);
    } else {
        mag.x = 0; mag.y = 0; mag.z = 0;
    }

    applyCalibration(accel, gyro, mag);
    return true;
}
