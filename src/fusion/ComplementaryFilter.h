#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "SensorFusion.h"

/**
 * @brief 6-DOF / 9-DOF Complementary AHRS Filter
 * 
 * Fuses high-frequency angular rate from gyroscope with low-frequency
 * orientation from accelerometer (and tilt-compensated magnetometer for yaw).
 * The weight parameter alpha determines the balance (typically 0.95 - 0.98).
 */
class ComplementaryFilter : public SensorFusion {
public:
    ComplementaryFilter(float alpha = 0.98f);

    void reset() override;

    void update6DOF(float gx, float gy, float gz, 
                    float ax, float ay, float az, 
                    float dt) override;

    void update9DOF(float gx, float gy, float gz, 
                    float ax, float ay, float az, 
                    float mx, float my, float mz, 
                    float dt) override;

    Quaternion getQuaternion() const override { return Quaternion(q0, q1, q2, q3); }
    EulerAngles getEuler() const override { return EulerAngles(_roll, _pitch, _yaw); }
    Vector3 getGravity() const override;
    Vector3 getLinearAccel(const Vector3& rawAccel) const override;

    void setAlpha(float alpha) { _alpha = alpha; }
    float getAlpha() const { return _alpha; }

    float getRoll() const { return _roll; }
    float getPitch() const { return _pitch; }
    float getYaw() const { return _yaw; }

private:
    float _alpha;
    float _roll;
    float _pitch;
    float _yaw;
    float q0, q1, q2, q3;
    bool  _initialized;

    void updateQuaternion();
};

#endif // COMPLEMENTARY_FILTER_H
