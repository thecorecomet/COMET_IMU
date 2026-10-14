#ifndef DRIVER_MPU6050_H
#define DRIVER_MPU6050_H

#include "../BaseIMUDriver.h"

class DriverMPU6050 : public BaseIMUDriver {
public:
    DriverMPU6050(BusInterface* bus);

    bool begin(const IMUConfig& config) override;
    bool readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) override;
    bool hasHardwareFusion() const override { return false; }
    bool is9DOF() const override { return false; }
    IMUType getType() const override { return IMU_MPU6050; }
    const char* getName() const override { return "InvenSense MPU-6050"; }

private:
    float _accelScaleFactor;
    float _gyroScaleFactor;
};

#endif // DRIVER_MPU6050_H
