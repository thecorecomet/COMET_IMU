#ifndef DRIVER_BNO055_H
#define DRIVER_BNO055_H

#include "../BaseIMUDriver.h"

class DriverBNO055 : public BaseIMUDriver {
public:
    DriverBNO055(BusInterface* bus);

    bool begin(const IMUConfig& config) override;
    bool readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) override;
    
    // Bosch BNO055 has built-in 9-DOF hardware sensor fusion on its internal Cortex-M0!
    bool hasHardwareFusion() const override { return true; }
    bool readHardwareFusion(Quaternion& q, Vector3& linAccel, Vector3& gravity) override;

    bool is9DOF() const override { return true; }
    IMUType getType() const override { return IMU_BNO055; }
    const char* getName() const override { return "Bosch BNO055 (Absolute Orientation)"; }

    uint8_t getCalibrationStatus();
    void setMode(uint8_t mode);

private:
    uint8_t _mode;
};

#endif // DRIVER_BNO055_H
