#ifndef DRIVER_ICM20948_H
#define DRIVER_ICM20948_H

#include "../BaseIMUDriver.h"

class DriverICM20948 : public BaseIMUDriver {
public:
    DriverICM20948(BusInterface* bus);

    bool begin(const IMUConfig& config) override;
    bool readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) override;
    bool hasHardwareFusion() const override { return false; }
    bool is9DOF() const override { return _hasMag; }
    IMUType getType() const override { return IMU_ICM20948; }
    const char* getName() const override { return "TDK InvenSense ICM-20948"; }

    void selectUserBank(uint8_t bank);

private:
    float _accelScaleFactor;
    float _gyroScaleFactor;
    bool _hasMag;
    uint8_t _currentBank;

    bool initAK09916();
    bool readMag(Vector3& mag);
};

#endif // DRIVER_ICM20948_H
