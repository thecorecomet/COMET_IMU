#ifndef DRIVER_BMI160_H
#define DRIVER_BMI160_H

#include "../BaseIMUDriver.h"

class DriverBMI160 : public BaseIMUDriver {
public:
    DriverBMI160(BusInterface* bus);

    bool begin(const IMUConfig& config) override;
    bool readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) override;
    bool hasHardwareFusion() const override { return false; }
    bool is9DOF() const override { return false; }
    IMUType getType() const override { return _isBMI270 ? IMU_BMI270 : IMU_BMI160; }
    const char* getName() const override { return _isBMI270 ? "Bosch BMI270" : "Bosch BMI160"; }

private:
    float _accelScaleFactor;
    float _gyroScaleFactor;
    bool _isBMI270;
};

#endif // DRIVER_BMI160_H
