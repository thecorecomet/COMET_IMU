#ifndef DRIVER_ICM42688_H
#define DRIVER_ICM42688_H

#include "../BaseIMUDriver.h"

class DriverICM42688 : public BaseIMUDriver {
public:
    DriverICM42688(BusInterface* bus);

    bool begin(const IMUConfig& config) override;
    bool readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) override;
    bool hasHardwareFusion() const override { return false; }
    bool is9DOF() const override { return false; }
    IMUType getType() const override { return IMU_ICM42688; }
    const char* getName() const override { return "TDK InvenSense ICM-42688"; }

private:
    float _accelScaleFactor;
    float _gyroScaleFactor;
};

#endif // DRIVER_ICM42688_H
