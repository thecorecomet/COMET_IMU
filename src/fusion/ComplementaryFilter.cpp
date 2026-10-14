#include "ComplementaryFilter.h"
#include <math.h>

ComplementaryFilter::ComplementaryFilter(float alpha) 
    : _alpha(alpha), _roll(0.0f), _pitch(0.0f), _yaw(0.0f),
      q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
      _initialized(false) {
    reset();
}

void ComplementaryFilter::reset() {
    _roll = 0.0f;
    _pitch = 0.0f;
    _yaw = 0.0f;
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    _initialized = false;
}

void ComplementaryFilter::update6DOF(float gx, float gy, float gz, 
                                     float ax, float ay, float az, 
                                     float dt) {
    if (dt <= 0.0f) return;

    if (ax == 0.0f && ay == 0.0f && az == 0.0f) {
        _roll  += gx * dt;
        _pitch += gy * dt;
        _yaw   += gz * dt;
        while (_roll > 180.0f) _roll -= 360.0f;
        while (_roll < -180.0f) _roll += 360.0f;
        while (_pitch > 180.0f) _pitch -= 360.0f;
        while (_pitch < -180.0f) _pitch += 360.0f;
        while (_yaw > 180.0f) _yaw -= 360.0f;
        while (_yaw < -180.0f) _yaw += 360.0f;
        updateQuaternion();
        return;
    }

    // Accelerometer tilt calculation (degrees)
    float rollAcc  = atan2f(ay, az) * (float)RAD_TO_DEG;
    float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * (float)RAD_TO_DEG;

    if (!_initialized) {
        _roll = rollAcc;
        _pitch = pitchAcc;
        _yaw = 0.0f;
        _initialized = true;
    } else {
        // Gyroscope angular rate integration
        float rollGyro  = _roll + gx * dt;
        float pitchGyro = _pitch + gy * dt;

        // Shortest arc angle difference
        float diffRoll = rollAcc - rollGyro;
        while (diffRoll > 180.0f) diffRoll -= 360.0f;
        while (diffRoll < -180.0f) diffRoll += 360.0f;

        float diffPitch = pitchAcc - pitchGyro;
        while (diffPitch > 180.0f) diffPitch -= 360.0f;
        while (diffPitch < -180.0f) diffPitch += 360.0f;

        // Complementary filter blend
        _roll  = rollGyro + (1.0f - _alpha) * diffRoll;
        _pitch = pitchGyro + (1.0f - _alpha) * diffPitch;

        while (_roll > 180.0f) _roll -= 360.0f;
        while (_roll < -180.0f) _roll += 360.0f;
        while (_pitch > 180.0f) _pitch -= 360.0f;
        while (_pitch < -180.0f) _pitch += 360.0f;

        // 6-DOF yaw integrated from gyro Z
        _yaw += gz * dt;
        while (_yaw > 180.0f) _yaw -= 360.0f;
        while (_yaw < -180.0f) _yaw += 360.0f;
    }

    updateQuaternion();
}

void ComplementaryFilter::update9DOF(float gx, float gy, float gz, 
                                     float ax, float ay, float az, 
                                     float mx, float my, float mz, 
                                     float dt) {
    if (dt <= 0.0f) return;

    if (mx == 0.0f && my == 0.0f && mz == 0.0f) {
        update6DOF(gx, gy, gz, ax, ay, az, dt);
        return;
    }

    if (ax == 0.0f && ay == 0.0f && az == 0.0f) {
        update6DOF(gx, gy, gz, ax, ay, az, dt);
        return;
    }

    // Accelerometer tilt calculation
    float rollAcc  = atan2f(ay, az) * (float)RAD_TO_DEG;
    float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * (float)RAD_TO_DEG;

    if (!_initialized) {
        _roll = rollAcc;
        _pitch = pitchAcc;
        _initialized = true;
    } else {
        float rollGyro  = _roll + gx * dt;
        float pitchGyro = _pitch + gy * dt;

        float diffRoll = rollAcc - rollGyro;
        while (diffRoll > 180.0f) diffRoll -= 360.0f;
        while (diffRoll < -180.0f) diffRoll += 360.0f;

        float diffPitch = pitchAcc - pitchGyro;
        while (diffPitch > 180.0f) diffPitch -= 360.0f;
        while (diffPitch < -180.0f) diffPitch += 360.0f;

        _roll  = rollGyro + (1.0f - _alpha) * diffRoll;
        _pitch = pitchGyro + (1.0f - _alpha) * diffPitch;

        while (_roll > 180.0f) _roll -= 360.0f;
        while (_roll < -180.0f) _roll += 360.0f;
        while (_pitch > 180.0f) _pitch -= 360.0f;
        while (_pitch < -180.0f) _pitch += 360.0f;
    }

    // Tilt-compensated magnetic heading
    float rollRad = _roll * (float)DEG_TO_RAD;
    float pitchRad = _pitch * (float)DEG_TO_RAD;
    float cr = cosf(rollRad);
    float sr = sinf(rollRad);
    float cp = cosf(pitchRad);
    float sp = sinf(pitchRad);

    float Xh = mx * cp + my * sp * sr + mz * sp * cr;
    float Yh = my * cr - mz * sr;
    float yawMag = atan2f(-Yh, Xh) * (float)RAD_TO_DEG;

    // Blend yaw with complementary filter
    float yawGyro = _yaw + gz * dt;
    float diffYaw = yawMag - yawGyro;
    while (diffYaw > 180.0f) diffYaw -= 360.0f;
    while (diffYaw < -180.0f) diffYaw += 360.0f;

    _yaw = yawGyro + (1.0f - _alpha) * diffYaw;
    while (_yaw > 180.0f) _yaw -= 360.0f;
    while (_yaw < -180.0f) _yaw += 360.0f;

    updateQuaternion();
}

void ComplementaryFilter::updateQuaternion() {
    float halfRoll  = (_roll  * (float)DEG_TO_RAD) * 0.5f;
    float halfPitch = (_pitch * (float)DEG_TO_RAD) * 0.5f;
    float halfYaw   = (_yaw   * (float)DEG_TO_RAD) * 0.5f;

    float cr = cosf(halfRoll);
    float sr = sinf(halfRoll);
    float cp = cosf(halfPitch);
    float sp = sinf(halfPitch);
    float cy = cosf(halfYaw);
    float sy = sinf(halfYaw);

    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;

    float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

Vector3 ComplementaryFilter::getGravity() const {
    Vector3 grav;
    grav.x = 2.0f * (q1 * q3 - q0 * q2);
    grav.y = 2.0f * (q0 * q1 + q2 * q3);
    grav.z = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    return grav;
}

Vector3 ComplementaryFilter::getLinearAccel(const Vector3& rawAccel) const {
    Vector3 grav = getGravity();
    return Vector3(
        rawAccel.x - grav.x * 9.80665f,
        rawAccel.y - grav.y * 9.80665f,
        rawAccel.z - grav.z * 9.80665f
    );
}
