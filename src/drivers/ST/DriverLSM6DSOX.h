#ifndef DRIVER_LSM6DSOX_H
#define DRIVER_LSM6DSOX_H

#include "../BaseIMUDriver.h"

class DriverLSM6DSOX : public BaseIMUDriver {
public:
    DriverLSM6DSOX(BusInterface* bus);

    bool begin(const IMUConfig& config) override;
    bool readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) override;
    bool hasHardwareFusion() const override { return false; }
    bool is9DOF() const override { return false; }
    IMUType getType() const override { return _isLSM6DS3 ? IMU_LSM6DS3 : IMU_LSM6DSOX; }
    const char* getName() const override { return _isLSM6DS3 ? "ST LSM6DS3" : "ST LSM6DSOX"; }

private:
    float _accelScaleFactor;
    float _gyroScaleFactor;
    bool _isLSM6DS3;
};

#endif // DRIVER_LSM6DSOX_H
