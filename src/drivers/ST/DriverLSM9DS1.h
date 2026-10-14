#ifndef DRIVER_LSM9DS1_H
#define DRIVER_LSM9DS1_H

#include "../BaseIMUDriver.h"

class DriverLSM9DS1 : public BaseIMUDriver {
public:
    DriverLSM9DS1(BusInterface* bus);

    bool begin(const IMUConfig& config) override;
    bool readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) override;
    bool hasHardwareFusion() const override { return false; }
    bool is9DOF() const override { return _hasMag; }
    IMUType getType() const override { return IMU_LSM9DS1; }
    const char* getName() const override { return "ST LSM9DS1 (9-DOF)"; }

    void setMagI2CAddress(uint8_t addr) { _magI2CAddr = addr; }

private:
    float _accelScaleFactor;
    float _gyroScaleFactor;
    float _magScaleFactor;
    uint8_t _magI2CAddr;
    bool _hasMag;

    bool initMag();
    bool readMag(Vector3& mag);
};

#endif // DRIVER_LSM9DS1_H
