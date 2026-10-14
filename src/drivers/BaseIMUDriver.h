#ifndef BASE_IMU_DRIVER_H
#define BASE_IMU_DRIVER_H

#include <Arduino.h>
#include "../IMUTypes.h"
#include "../bus/BusInterface.h"

class BaseIMUDriver {
public:
    BaseIMUDriver(BusInterface* bus) : _bus(bus), _isCalibrated(false) {
        gyroOffset = Vector3(0, 0, 0);
        accelOffset = Vector3(0, 0, 0);
        accelScale = Vector3(1, 1, 1);
        magOffset = Vector3(0, 0, 0);
        magScale = Vector3(1, 1, 1);
    }

    virtual ~BaseIMUDriver() {}

    virtual bool begin(const IMUConfig& config) = 0;
    virtual bool readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) = 0;
    virtual bool hasHardwareFusion() const = 0;
    virtual bool readHardwareFusion(Quaternion& q, Vector3& linAccel, Vector3& gravity) {
        (void)q; (void)linAccel; (void)gravity;
        return false;
    }

    virtual bool is9DOF() const = 0;
    virtual IMUType getType() const = 0;
    virtual const char* getName() const = 0;

    BusInterface* getBus() const { return _bus; }
    void setBus(BusInterface* bus) { _bus = bus; }

    // Automated Gyroscope Calibration (measures still bias)
    virtual void calibrateGyro(uint16_t samples = 300, uint16_t delayMs = 4) {
        Vector3 sum(0, 0, 0);
        Vector3 a, g, m;
        float t;

        for (uint16_t i = 0; i < samples; i++) {
            if (readRaw(a, g, m, t)) {
                // Remove existing offset if any was applied
                sum.x += (g.x + gyroOffset.x);
                sum.y += (g.y + gyroOffset.y);
                sum.z += (g.z + gyroOffset.z);
            }
            delay(delayMs);
        }

        gyroOffset.x = sum.x / (float)samples;
        gyroOffset.y = sum.y / (float)samples;
        gyroOffset.z = sum.z / (float)samples;
        _isCalibrated = true;
    }

    // Automated Accelerometer Level Calibration (assumes Z-axis is pointing UP at 1G)
    virtual void calibrateAccel(uint16_t samples = 300, uint16_t delayMs = 4) {
        Vector3 sum(0, 0, 0);
        Vector3 a, g, m;
        float t;

        for (uint16_t i = 0; i < samples; i++) {
            if (readRaw(a, g, m, t)) {
                sum.x += (a.x + accelOffset.x);
                sum.y += (a.y + accelOffset.y);
                sum.z += (a.z + accelOffset.z);
            }
            delay(delayMs);
        }

        accelOffset.x = sum.x / (float)samples;
        accelOffset.y = sum.y / (float)samples;
        // Assume resting on flat surface with gravity along Z axis (+9.80665 m/s^2)
        accelOffset.z = (sum.z / (float)samples) - 9.80665f;
    }

    void setGyroOffset(const Vector3& offset) { gyroOffset = offset; }
    Vector3 getGyroOffset() const { return gyroOffset; }

    void setAccelOffset(const Vector3& offset) { accelOffset = offset; }
    Vector3 getAccelOffset() const { return accelOffset; }

    void setMagCalibration(const Vector3& offset, const Vector3& scale) {
        magOffset = offset;
        magScale = scale;
    }

protected:
    BusInterface* _bus;
    Vector3 gyroOffset;
    Vector3 accelOffset;
    Vector3 accelScale;
    Vector3 magOffset;
    Vector3 magScale;
    bool _isCalibrated;

    void applyCalibration(Vector3& a, Vector3& g, Vector3& m) {
        g.x -= gyroOffset.x;
        g.y -= gyroOffset.y;
        g.z -= gyroOffset.z;

        a.x = (a.x - accelOffset.x) * accelScale.x;
        a.y = (a.y - accelOffset.y) * accelScale.y;
        a.z = (a.z - accelOffset.z) * accelScale.z;

        m.x = (m.x - magOffset.x) * magScale.x;
        m.y = (m.y - magOffset.y) * magScale.y;
        m.z = (m.z - magOffset.z) * magScale.z;
    }
};

#endif // BASE_IMU_DRIVER_H
