#include "DriverICM20948.h"
#include "../../bus/I2CBus.h"

#define REG_BANK_SEL   0x7F
#define AK09916_ADDR   0x0C
#define AK09916_WIA2   0x01
#define AK09916_HXL    0x11
#define AK09916_CNTL2  0x31

DriverICM20948::DriverICM20948(BusInterface* bus)
    : BaseIMUDriver(bus), 
      _accelScaleFactor(9.80665f / 8192.0f), 
      _gyroScaleFactor(1.0f / 32.8f), 
      _hasMag(false),
      _currentBank(0xFF) {}

void DriverICM20948::selectUserBank(uint8_t bank) {
    if (_currentBank != bank && _bus) {
        _bus->writeByte(REG_BANK_SEL, (bank << 4) & 0x30);
        _currentBank = bank;
    }
}

bool DriverICM20948::begin(const IMUConfig& config) {
    if (!_bus) return false;
    _bus->begin();

    // Select Bank 0
    selectUserBank(0);

    // Reset ICM-20948
    _bus->writeByte(0x06, 0x80); // PWR_MGMT_1 reset
    delay(50);

    // Auto-select clock
    _bus->writeByte(0x06, 0x01);
    delay(10);

    // Enable Accel and Gyro
    _bus->writeByte(0x07, 0x00); // PWR_MGMT_2: enable all axes

    // Configure Gyro (Bank 2, Reg 0x01)
    selectUserBank(2);
    uint8_t gyroConfig = 0x05; // Default 1000 dps, DLPF enabled
    if (config.gyroFsrDps <= 250) {
        gyroConfig = 0x01;
        _gyroScaleFactor = 1.0f / 131.0f;
    } else if (config.gyroFsrDps <= 500) {
        gyroConfig = 0x03;
        _gyroScaleFactor = 1.0f / 65.5f;
    } else if (config.gyroFsrDps <= 1000) {
        gyroConfig = 0x05;
        _gyroScaleFactor = 1.0f / 32.8f;
    } else {
        gyroConfig = 0x07;
        _gyroScaleFactor = 1.0f / 16.4f;
    }
    _bus->writeByte(0x01, gyroConfig);

    // Configure Accel (Bank 2, Reg 0x14)
    uint8_t accelConfig = 0x03; // Default +/- 4G, DLPF enabled
    if (config.accelFsrG <= 2) {
        accelConfig = 0x01;
        _accelScaleFactor = 9.80665f / 16384.0f;
    } else if (config.accelFsrG <= 4) {
        accelConfig = 0x03;
        _accelScaleFactor = 9.80665f / 8192.0f;
    } else if (config.accelFsrG <= 8) {
        accelConfig = 0x05;
        _accelScaleFactor = 9.80665f / 4096.0f;
    } else {
        accelConfig = 0x07;
        _accelScaleFactor = 9.80665f / 2048.0f;
    }
    _bus->writeByte(0x14, accelConfig);

    // Return to Bank 0
    selectUserBank(0);

    if (config.enableMag) {
        _hasMag = initAK09916();
    }

    return true;
}

bool DriverICM20948::initAK09916() {
    if (_bus->getType() == BUS_I2C) {
        selectUserBank(0);
        // Enable I2C bypass mode (INT_PIN_CFG reg 0x0F, bit 1 = 0x02)
        _bus->writeByte(0x0F, 0x02);
        // Disable I2C master in USER_CTRL (0x03)
        _bus->writeByte(0x03, 0x00);
        delay(10);

        I2CBus* i2c = static_cast<I2CBus*>(_bus);
        TwoWire* wire = i2c->getWire();

        // Check AK09916 WIA2 (0x01) == 0x09
        wire->beginTransmission(AK09916_ADDR);
        wire->write(AK09916_WIA2);
        if (wire->endTransmission() != 0) return false;

        wire->requestFrom((uint8_t)AK09916_ADDR, (uint8_t)1);
        if (!wire->available() || wire->read() != 0x09) return false;

        // Reset AK09916 (CNTL3 = 0x01)
        wire->beginTransmission(AK09916_ADDR);
        wire->write(0x32);
        wire->write(0x01);
        wire->endTransmission();
        delay(20);

        // Continuous measurement mode 4 (100Hz): CNTL2 = 0x08
        wire->beginTransmission(AK09916_ADDR);
        wire->write(AK09916_CNTL2);
        wire->write(0x08);
        wire->endTransmission();
        delay(10);

        return true;
    }
    return false;
}

bool DriverICM20948::readMag(Vector3& mag) {
    if (!_hasMag || _bus->getType() != BUS_I2C) {
        mag.x = 0; mag.y = 0; mag.z = 0;
        return false;
    }

    I2CBus* i2c = static_cast<I2CBus*>(_bus);
    TwoWire* wire = i2c->getWire();

    wire->beginTransmission(AK09916_ADDR);
    wire->write(AK09916_HXL);
    if (wire->endTransmission() != 0) return false;

    // Read 6 bytes of data + 2 bytes status (ST1/ST2)
    wire->requestFrom((uint8_t)AK09916_ADDR, (uint8_t)8);
    if (wire->available() < 8) return false;

    uint8_t x_l = wire->read();
    uint8_t x_h = wire->read();
    uint8_t y_l = wire->read();
    uint8_t y_h = wire->read();
    uint8_t z_l = wire->read();
    uint8_t z_h = wire->read();
    (void)wire->read(); // dummy
    uint8_t st2 = wire->read();

    if (st2 & 0x08) return false; // Overflow check

    int16_t rawMx = (int16_t)((x_h << 8) | x_l);
    int16_t rawMy = (int16_t)((y_h << 8) | y_l);
    int16_t rawMz = (int16_t)((z_h << 8) | z_l);

    // 0.15 uT per LSB
    const float magRes = 0.15f;
    mag.x = (float)rawMx * magRes;
    mag.y = -(float)rawMy * magRes;
    mag.z = -(float)rawMz * magRes;

    return true;
}

bool DriverICM20948::readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) {
    if (!_bus) return false;

    selectUserBank(0);

    // Read 14 bytes: Accel (6), Gyro (6), Temp (2) starting from ACCEL_XOUT_H (0x2D)
    uint8_t buffer[14];
    if (!_bus->readBytes(0x2D, buffer, 14)) {
        return false;
    }

    int16_t rawAx = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t rawAy = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t rawAz = (int16_t)((buffer[4] << 8) | buffer[5]);

    int16_t rawGx = (int16_t)((buffer[6] << 8) | buffer[7]);
    int16_t rawGy = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t rawGz = (int16_t)((buffer[10] << 8) | buffer[11]);

    int16_t rawTemp = (int16_t)((buffer[12] << 8) | buffer[13]);

    accel.x = (float)rawAx * _accelScaleFactor;
    accel.y = (float)rawAy * _accelScaleFactor;
    accel.z = (float)rawAz * _accelScaleFactor;

    gyro.x = (float)rawGx * _gyroScaleFactor;
    gyro.y = (float)rawGy * _gyroScaleFactor;
    gyro.z = (float)rawGz * _gyroScaleFactor;

    temp = ((float)rawTemp - 21.0f) / 333.87f + 21.0f;

    if (_hasMag) {
        readMag(mag);
    } else {
        mag.x = 0; mag.y = 0; mag.z = 0;
    }

    applyCalibration(accel, gyro, mag);
    return true;
}
