#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "SensorFusion.h"

/**
 * @brief 1-Axis 2-State Linear Kalman Filter
 * 
 * Estimates state [angle, gyro_bias]^T using angular rate from gyroscope
 * and angle measurement from accelerometer / magnetometer.
 */
struct Kalman1D {
    float angle;      // Filtered angle (degrees)
    float bias;       // Estimated gyroscope bias (deg/s)
    float rate;       // Unbiased angular rate (deg/s)
    float P[2][2];    // Error covariance matrix
    float Q_angle;    // Process noise variance for angle
    float Q_bias;     // Process noise variance for gyro bias
    float R_measure;  // Measurement noise variance

    Kalman1D() { reset(); }

    void reset() {
        angle = 0.0f;
        bias = 0.0f;
        rate = 0.0f;
        P[0][0] = 1.0f; P[0][1] = 0.0f;
        P[1][0] = 0.0f; P[1][1] = 1.0f;
        Q_angle = 0.001f;
        Q_bias = 0.003f;
        R_measure = 0.03f;
    }

    void setAngle(float a) { angle = a; }
    float getRate() const { return rate; }
    float getBias() const { return bias; }
    void setQangle(float q) { Q_angle = q; }
    void setQbias(float q) { Q_bias = q; }
    void setRmeasure(float r) { R_measure = r; }

    // Update state with measurement and gyro angular rate
    float update(float newAngle, float newRate, float dt) {
        if (dt <= 0.0f) return angle;

        // Predict step: angular rate without estimated bias
        rate = newRate - bias;
        angle += dt * rate;

        // Update error covariance matrix: P = F * P * F^T + Q
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // Innovation with shortest path wrapping [-180, +180]
        float y = newAngle - angle;
        while (y > 180.0f) y -= 360.0f;
        while (y < -180.0f) y += 360.0f;

        // Measurement update step: S = H * P * H^T + R
        float S = P[0][0] + R_measure;
        float K0 = P[0][0] / S;
        float K1 = P[1][0] / S;

        angle += K0 * y;
        bias  += K1 * y;

        // Wrap angle to [-180, 180]
        while (angle > 180.0f) angle -= 360.0f;
        while (angle < -180.0f) angle += 360.0f;

        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K0 * P00_temp;
        P[0][1] -= K0 * P01_temp;
        P[1][0] -= K1 * P00_temp;
        P[1][1] -= K1 * P01_temp;

        return angle;
    }
};

/**
 * @brief 6-DOF / 9-DOF Kalman AHRS Filter
 * 
 * Provides fused orientation as Quaternion and Euler Angles (Roll, Pitch, Yaw),
 * as well as gravity vector and linear dynamic acceleration.
 */
class KalmanFilter : public SensorFusion {
public:
    KalmanFilter(float qAngle = 0.001f, float qBias = 0.003f, float rMeasure = 0.03f);

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

    void setQangle(float qAngle);
    void setQbias(float qBias);
    void setRmeasure(float rMeasure);

    float getRoll() const { return _roll; }
    float getPitch() const { return _pitch; }
    float getYaw() const { return _yaw; }

    Kalman1D& getKalmanRoll() { return _kalmanRoll; }
    Kalman1D& getKalmanPitch() { return _kalmanPitch; }
    Kalman1D& getKalmanYaw() { return _kalmanYaw; }

private:
    Kalman1D _kalmanRoll;
    Kalman1D _kalmanPitch;
    Kalman1D _kalmanYaw;

    float _roll;
    float _pitch;
    float _yaw;
    float q0, q1, q2, q3;
    bool  _initialized;

    void updateQuaternion();
};

#endif // KALMAN_FILTER_H
