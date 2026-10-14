#ifndef DRIVER_MPU9250_H
#define DRIVER_MPU9250_H

#include "../BaseIMUDriver.h"

class DriverMPU9250 : public BaseIMUDriver {
public:
    DriverMPU9250(BusInterface* bus);

    bool begin(const IMUConfig& config) override;
    bool readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) override;
    bool hasHardwareFusion() const override { return false; }
    bool is9DOF() const override { return _hasMag; }
    IMUType getType() const override { return _hasMag ? IMU_MPU9250 : IMU_MPU6500; }
    const char* getName() const override { return _hasMag ? "InvenSense MPU-9250" : "InvenSense MPU-6500"; }

private:
    float _accelScaleFactor;
    float _gyroScaleFactor;
    bool _hasMag;
    float _magAdj[3];

    bool initAK8963();
    bool readMag(Vector3& mag);
};

#endif // DRIVER_MPU9250_H
