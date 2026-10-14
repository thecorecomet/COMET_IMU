#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <Arduino.h>
#include <math.h>

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886
#endif

enum IMUType {
    IMU_UNKNOWN = 0,
    IMU_MPU6050,
    IMU_MPU6500,
    IMU_MPU9250,
    IMU_ICM20948,
    IMU_ICM42688,
    IMU_BNO055,
    IMU_BNO080,
    IMU_BNO085,
    IMU_BMI160,
    IMU_BMI270,
    IMU_LSM6DSOX,
    IMU_LSM6DS3,
    IMU_LSM9DS1
};

enum FusionMode {
    FUSION_AUTO = 0,     // Automatically offload to hardware fusion if sensor supports it, else Madgwick
    FUSION_HARDWARE,     // Require on-chip hardware fusion (BNO055, BNO08x, DMP)
    FUSION_MADGWICK,     // Software Madgwick 6-DOF / 9-DOF AHRS filter
    FUSION_MAHONY,       // Software Mahony 6-DOF / 9-DOF AHRS filter
    FUSION_KALMAN,       // Software Kalman 6-DOF / 9-DOF AHRS filter
    FUSION_COMPLEMENTARY,// Software Complementary 6-DOF / 9-DOF AHRS filter
    FUSION_OFF           // Raw data only, no orientation computation
};

enum BusType {
    BUS_I2C = 0,
    BUS_SPI = 1
};

struct Vector3 {
    float x;
    float y;
    float z;

    Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

    float lengthSq() const { return x * x + y * y + z * z; }
    float length() const { return sqrtf(lengthSq()); }

    void normalize() {
        float len = length();
        if (len > 1e-6f) {
            float inv = 1.0f / len;
            x *= inv;
            y *= inv;
            z *= inv;
        }
    }

    Vector3 operator+(const Vector3& o) const { return Vector3(x + o.x, y + o.y, z + o.z); }
    Vector3 operator-(const Vector3& o) const { return Vector3(x - o.x, y - o.y, z - o.z); }
    Vector3 operator*(float s) const { return Vector3(x * s, y * s, z * s); }
    Vector3& operator+=(const Vector3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vector3& operator-=(const Vector3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    static float dot(const Vector3& a, const Vector3& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static Vector3 cross(const Vector3& a, const Vector3& b) {
        return Vector3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );
    }
};

struct EulerAngles {
    float roll;   // Rotation around X axis (degrees)
    float pitch;  // Rotation around Y axis (degrees)
    float yaw;    // Rotation around Z axis (degrees)

    EulerAngles() : roll(0.0f), pitch(0.0f), yaw(0.0f) {}
    EulerAngles(float r, float p, float y) : roll(r), pitch(p), yaw(y) {}
};

struct Quaternion {
    float w;
    float x;
    float y;
    float z;

    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quaternion(float _w, float _x, float _y, float _z) : w(_w), x(_x), y(_y), z(_z) {}

    void normalize() {
        float norm = sqrtf(w * w + x * x + y * y + z * z);
        if (norm > 1e-6f) {
            float inv = 1.0f / norm;
            w *= inv;
            x *= inv;
            y *= inv;
            z *= inv;
        } else {
            w = 1.0f; x = 0.0f; y = 0.0f; z = 0.0f;
        }
    }

    // Convert to Euler angles (Roll, Pitch, Yaw) in degrees
    EulerAngles toEuler() const {
        EulerAngles e;

        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        e.roll = atan2f(sinr_cosp, cosr_cosp) * (float)RAD_TO_DEG;

        // Pitch (y-axis rotation)
        float sinp = 2.0f * (w * y - z * x);
        if (fabsf(sinp) >= 1.0f) {
            e.pitch = copysignf(90.0f, sinp); // Use 90 degrees if out of range
        } else {
            e.pitch = asinf(sinp) * (float)RAD_TO_DEG;
        }

        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        e.yaw = atan2f(siny_cosp, cosy_cosp) * (float)RAD_TO_DEG;

        return e;
    }

    // Rotate a vector by this quaternion
    Vector3 rotateVector(const Vector3& v) const {
        Vector3 qv(x, y, z);
        Vector3 uv = Vector3::cross(qv, v);
        Vector3 uuv = Vector3::cross(qv, uv);
        return v + ((uv * w) + uuv) * 2.0f;
    }
};

struct SensorData {
    Vector3 accel;          // Accelerometer in m/s^2
    Vector3 gyro;           // Gyroscope in deg/s
    Vector3 mag;            // Magnetometer in microTesla (uT)
    float temperature;      // Temperature in Celsius

    Quaternion orientation; // Estimated orientation quaternion (W, X, Y, Z)
    EulerAngles euler;      // Estimated Roll, Pitch, Yaw in degrees
    Vector3 linearAccel;    // Linear acceleration (accel minus gravity)
    Vector3 gravity;        // Estimated direction of gravity in sensor frame

    uint32_t timestampMicros;
    bool hasMag;
    bool hasHardwareFusion;

    SensorData() : temperature(0.0f), timestampMicros(0), hasMag(false), hasHardwareFusion(false) {}
};

struct IMUConfig {
    uint16_t sampleRateHz;  // Desired sample rate (e.g. 100Hz, 200Hz)
    uint8_t accelFsrG;      // Accel Full Scale Range in G (2, 4, 8, 16)
    uint16_t gyroFsrDps;    // Gyro Full Scale Range in DPS (250, 500, 1000, 2000)
    bool enableMag;         // Enable magnetometer if present
    FusionMode fusionMode;  // Desired fusion mode

    IMUConfig() : 
        sampleRateHz(100), 
        accelFsrG(4), 
        gyroFsrDps(1000), 
        enableMag(true), 
        fusionMode(FUSION_AUTO) {}
};

#endif // IMU_TYPES_H
