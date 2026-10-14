#ifndef MAHONY_FILTER_H
#define MAHONY_FILTER_H

#include "SensorFusion.h"

class MahonyFilter : public SensorFusion {
public:
    MahonyFilter(float kp = 2.0f, float ki = 0.005f);

    void reset() override;

    void update6DOF(float gx, float gy, float gz, 
                    float ax, float ay, float az, 
                    float dt) override;

    void update9DOF(float gx, float gy, float gz, 
                    float ax, float ay, float az, 
                    float mx, float my, float mz, 
                    float dt) override;

    Quaternion getQuaternion() const override { return Quaternion(q0, q1, q2, q3); }
    EulerAngles getEuler() const override;
    Vector3 getGravity() const override;
    Vector3 getLinearAccel(const Vector3& rawAccel) const override;

    void setKp(float kp) { _kp = kp; }
    void setKi(float ki) { _ki = ki; }

private:
    float _kp;
    float _ki;
    float q0, q1, q2, q3;
    float integralFBx, integralFBy, integralFBz;
};

#endif // MAHONY_FILTER_H
