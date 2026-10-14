#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "../IMUTypes.h"

class SensorFusion {
public:
    virtual ~SensorFusion() {}

    virtual void reset() = 0;
    
    // 6-DOF update (accel + gyro)
    virtual void update6DOF(float gx, float gy, float gz, 
                            float ax, float ay, float az, 
                            float dt) = 0;

    // 9-DOF update (accel + gyro + mag)
    virtual void update9DOF(float gx, float gy, float gz, 
                            float ax, float ay, float az, 
                            float mx, float my, float mz, 
                            float dt) = 0;

    virtual Quaternion getQuaternion() const = 0;
    virtual EulerAngles getEuler() const = 0;
    virtual Vector3 getGravity() const = 0;
    virtual Vector3 getLinearAccel(const Vector3& rawAccel) const = 0;

protected:
    static float invSqrt(float x) {
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i >> 1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        y = y * (1.5f - (halfx * y * y));
        return y;
    }
};

#endif // SENSOR_FUSION_H
