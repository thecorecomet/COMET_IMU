#include "DriverMPU9250.h"
#include "../../bus/I2CBus.h"

#define AK8963_I2C_ADDR   0x0C
#define AK8963_WIA        0x00
#define AK8963_HXL        0x03
#define AK8963_CNTL1      0x0A
#define AK8963_ASAX       0x10

DriverMPU9250::DriverMPU9250(BusInterface* bus)
    : BaseIMUDriver(bus), 
      _accelScaleFactor(9.80665f / 8192.0f), 
      _gyroScaleFactor(1.0f / 32.8f), 
      _hasMag(false) {
    _magAdj[0] = 1.0f;
    _magAdj[1] = 1.0f;
    _magAdj[2] = 1.0f;
}

bool DriverMPU9250::begin(const IMUConfig& config) {
    if (!_bus) return false;
    _bus->begin();

    // Reset device
    _bus->writeByte(0x6B, 0x80);
    delay(50);

    // Auto-select best clock source (PLL)
    _bus->writeByte(0x6B, 0x01);
    delay(10);

    _bus->writeByte(0x6C, 0x00); // Enable Accel & Gyro

    // Sample rate divider
    _bus->writeByte(0x19, 0x04); // 200Hz

    // DLPF ~44Hz
    _bus->writeByte(0x1A, 0x03);

    // Gyro Full Scale Range
    uint8_t gyroConfigVal = 0x10;
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

    // Accel Full Scale Range
    uint8_t accelConfigVal = 0x08;
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

    if (config.enableMag) {
        _hasMag = initAK8963();
    }

    return true;
}

bool DriverMPU9250::initAK8963() {
    if (_bus->getType() == BUS_I2C) {
        // Enable I2C bypass mode so MCU Wire can access AK8963 directly at 0x0C
        _bus->writeByte(0x37, 0x02); // BYPASS_EN = 1
        _bus->writeByte(0x6A, 0x00); // USER_CTRL: disable I2C master
        delay(10);

        I2CBus* i2c = static_cast<I2CBus*>(_bus);
        TwoWire* wire = i2c->getWire();

        // Check AK8963 WHO_AM_I
        wire->beginTransmission(AK8963_I2C_ADDR);
        wire->write(AK8963_WIA);
        if (wire->endTransmission() != 0) return false;

        wire->requestFrom((uint8_t)AK8963_I2C_ADDR, (uint8_t)1);
        if (!wire->available() || wire->read() != 0x48) return false;

        // Enter Fuse ROM access mode to read factory sensitivity adjustments
        wire->beginTransmission(AK8963_I2C_ADDR);
        wire->write(AK8963_CNTL1);
        wire->write(0x1F); // Fuse ROM mode
        wire->endTransmission();
        delay(10);

        wire->beginTransmission(AK8963_I2C_ADDR);
        wire->write(AK8963_ASAX);
        wire->endTransmission();
        wire->requestFrom((uint8_t)AK8963_I2C_ADDR, (uint8_t)3);
        if (wire->available() >= 3) {
            uint8_t asax = wire->read();
            uint8_t asay = wire->read();
            uint8_t asaz = wire->read();
            _magAdj[0] = (((float)asax - 128.0f) * 0.5f / 128.0f) + 1.0f;
            _magAdj[1] = (((float)asay - 128.0f) * 0.5f / 128.0f) + 1.0f;
            _magAdj[2] = (((float)asaz - 128.0f) * 0.5f / 128.0f) + 1.0f;
        }

        // Power down AK8963
        wire->beginTransmission(AK8963_I2C_ADDR);
        wire->write(AK8963_CNTL1);
        wire->write(0x00);
        wire->endTransmission();
        delay(10);

        // Continuous measurement mode 2 (100Hz, 16-bit)
        wire->beginTransmission(AK8963_I2C_ADDR);
        wire->write(AK8963_CNTL1);
        wire->write(0x16);
        wire->endTransmission();
        delay(10);

        return true;
    }
    return false;
}

bool DriverMPU9250::readMag(Vector3& mag) {
    if (!_hasMag || _bus->getType() != BUS_I2C) {
        mag.x = 0; mag.y = 0; mag.z = 0;
        return false;
    }

    I2CBus* i2c = static_cast<I2CBus*>(_bus);
    TwoWire* wire = i2c->getWire();

    wire->beginTransmission(AK8963_I2C_ADDR);
    wire->write(AK8963_HXL);
    if (wire->endTransmission() != 0) return false;

    // Read 7 bytes: X, Y, Z (little-endian) + ST2 status register
    wire->requestFrom((uint8_t)AK8963_I2C_ADDR, (uint8_t)7);
    if (wire->available() < 7) return false;

    uint8_t x_l = wire->read();
    uint8_t x_h = wire->read();
    uint8_t y_l = wire->read();
    uint8_t y_h = wire->read();
    uint8_t z_l = wire->read();
    uint8_t z_h = wire->read();
    uint8_t st2 = wire->read();

    // Check for magnetic sensor overflow
    if (st2 & 0x08) return false;

    int16_t rawMx = (int16_t)((x_h << 8) | x_l);
    int16_t rawMy = (int16_t)((y_h << 8) | y_l);
    int16_t rawMz = (int16_t)((z_h << 8) | z_l);

    // Convert to microTesla (uT): 4912 uT full scale / 32760 LSB = 0.1499 uT/LSB
    const float magRes = 4912.0f / 32760.0f;
    // Align AK8963 axes to MPU-9250 body frame: X -> Y, Y -> X, Z -> -Z
    mag.x = (float)rawMy * magRes * _magAdj[1];
    mag.y = (float)rawMx * magRes * _magAdj[0];
    mag.z = -(float)rawMz * magRes * _magAdj[2];

    return true;
}

bool DriverMPU9250::readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) {
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

    temp = ((float)rawTemp) / 333.87f + 21.0f;

    if (_hasMag) {
        readMag(mag);
    } else {
        mag.x = 0; mag.y = 0; mag.z = 0;
    }

    applyCalibration(accel, gyro, mag);
    return true;
}
