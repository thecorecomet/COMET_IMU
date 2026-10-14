#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include "SensorFusion.h"

class MadgwickFilter : public SensorFusion {
public:
    MadgwickFilter(float beta = 0.1f);

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

    void setBeta(float beta) { _beta = beta; }
    float getBeta() const { return _beta; }

private:
    float _beta;
    float q0, q1, q2, q3;
};

#endif // MADGWICK_FILTER_H
