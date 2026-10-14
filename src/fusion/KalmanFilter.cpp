#include "KalmanFilter.h"
#include <math.h>

KalmanFilter::KalmanFilter(float qAngle, float qBias, float rMeasure) 
    : _roll(0.0f), _pitch(0.0f), _yaw(0.0f),
      q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
      _initialized(false) {
    setQangle(qAngle);
    setQbias(qBias);
    setRmeasure(rMeasure);
    reset();
}

void KalmanFilter::reset() {
    _kalmanRoll.reset();
    _kalmanPitch.reset();
    _kalmanYaw.reset();
    _roll = 0.0f;
    _pitch = 0.0f;
    _yaw = 0.0f;
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    _initialized = false;
}

void KalmanFilter::setQangle(float qAngle) {
    _kalmanRoll.setQangle(qAngle);
    _kalmanPitch.setQangle(qAngle);
    _kalmanYaw.setQangle(qAngle);
}

void KalmanFilter::setQbias(float qBias) {
    _kalmanRoll.setQbias(qBias);
    _kalmanPitch.setQbias(qBias);
    _kalmanYaw.setQbias(qBias);
}

void KalmanFilter::setRmeasure(float rMeasure) {
    _kalmanRoll.setRmeasure(rMeasure);
    _kalmanPitch.setRmeasure(rMeasure);
    _kalmanYaw.setRmeasure(rMeasure);
}

void KalmanFilter::update6DOF(float gx, float gy, float gz, 
                              float ax, float ay, float az, 
                              float dt) {
    if (dt <= 0.0f) return;

    // If accelerometer is zero/invalid, integrate gyroscope only
    if (ax == 0.0f && ay == 0.0f && az == 0.0f) {
        _roll  += (gx - _kalmanRoll.getBias()) * dt;
        _pitch += (gy - _kalmanPitch.getBias()) * dt;
        _yaw   += (gz - _kalmanYaw.getBias()) * dt;
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
    // Roll around X axis: atan2(ay, az)
    // Pitch around Y axis: atan2(-ax, sqrt(ay^2 + az^2))
    float rollAcc  = atan2f(ay, az) * (float)RAD_TO_DEG;
    float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * (float)RAD_TO_DEG;

    if (!_initialized) {
        _kalmanRoll.setAngle(rollAcc);
        _kalmanPitch.setAngle(pitchAcc);
        _roll = rollAcc;
        _pitch = pitchAcc;
        _yaw = 0.0f;
        _initialized = true;
    } else {
        _roll  = _kalmanRoll.update(rollAcc, gx, dt);
        _pitch = _kalmanPitch.update(pitchAcc, gy, dt);

        // In 6-DOF, yaw is updated via gyro Z integration with bias subtraction
        float yawRate = gz - _kalmanYaw.getBias();
        _yaw += yawRate * dt;
        while (_yaw > 180.0f) _yaw -= 360.0f;
        while (_yaw < -180.0f) _yaw += 360.0f;
    }

    updateQuaternion();
}

void KalmanFilter::update9DOF(float gx, float gy, float gz, 
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

    float rollAcc  = atan2f(ay, az) * (float)RAD_TO_DEG;
    float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * (float)RAD_TO_DEG;

    if (!_initialized) {
        _kalmanRoll.setAngle(rollAcc);
        _kalmanPitch.setAngle(pitchAcc);
        _roll = rollAcc;
        _pitch = pitchAcc;
        _initialized = true;
    } else {
        _roll  = _kalmanRoll.update(rollAcc, gx, dt);
        _pitch = _kalmanPitch.update(pitchAcc, gy, dt);
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

    _yaw = _kalmanYaw.update(yawMag, gz, dt);

    updateQuaternion();
}

void KalmanFilter::updateQuaternion() {
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

Vector3 KalmanFilter::getGravity() const {
    Vector3 grav;
    grav.x = 2.0f * (q1 * q3 - q0 * q2);
    grav.y = 2.0f * (q0 * q1 + q2 * q3);
    grav.z = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    return grav;
}

Vector3 KalmanFilter::getLinearAccel(const Vector3& rawAccel) const {
    Vector3 grav = getGravity();
    return Vector3(
        rawAccel.x - grav.x * 9.80665f,
        rawAccel.y - grav.y * 9.80665f,
        rawAccel.z - grav.z * 9.80665f
    );
}
