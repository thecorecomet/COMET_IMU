#include "DriverBNO055.h"

#define BNO055_OPR_MODE_CONFIG   0x00
#define BNO055_OPR_MODE_IMU      0x08
#define BNO055_OPR_MODE_NDOF     0x0C

DriverBNO055::DriverBNO055(BusInterface* bus) 
    : BaseIMUDriver(bus), _mode(BNO055_OPR_MODE_NDOF) {}

void DriverBNO055::setMode(uint8_t mode) {
    if (!_bus) return;
    _mode = mode;
    _bus->writeByte(0x3D, _mode);
    delay(30); // Mode switch settling time per Bosch spec
}

bool DriverBNO055::begin(const IMUConfig& config) {
    if (!_bus) return false;
    _bus->begin();

    // Make sure we are in CONFIG mode to write registers
    _bus->writeByte(0x3D, BNO055_OPR_MODE_CONFIG);
    delay(25);

    // Reset BNO055
    _bus->writeByte(0x3F, 0x20); // SYS_TRIGGER bit 5 = RST_SYS
    delay(650);                  // Boot time after reset

    // Verify Chip ID
    uint8_t chipId = _bus->readByte(0x00);
    if (chipId != 0xA0) {
        return false;
    }

    // Set normal power mode (PWR_MODE reg 0x3E = 0x00)
    _bus->writeByte(0x3E, 0x00);
    delay(10);

    // Select page 0
    _bus->writeByte(0x07, 0x00);

    // Set units: m/s^2 for accel, dps for gyro, degrees for euler, Celsius for temp
    _bus->writeByte(0x3B, 0x00);

    // Use internal oscillator (or crystal if stable)
    _bus->writeByte(0x3F, 0x00);
    delay(10);

    // Choose NDOF (9-DOF full fusion) or IMU (6-DOF fusion without mag)
    uint8_t targetMode = (config.enableMag) ? BNO055_OPR_MODE_NDOF : BNO055_OPR_MODE_IMU;
    setMode(targetMode);

    return true;
}

uint8_t DriverBNO055::getCalibrationStatus() {
    if (!_bus) return 0;
    return _bus->readByte(0x35);
}

bool DriverBNO055::readHardwareFusion(Quaternion& q, Vector3& linAccel, Vector3& gravity) {
    if (!_bus) return false;

    // Read 8 bytes for Quaternion: W, X, Y, Z from 0x20
    uint8_t qBuf[8];
    if (!_bus->readBytes(0x20, qBuf, 8)) {
        return false;
    }

    int16_t qw = (int16_t)((qBuf[1] << 8) | qBuf[0]);
    int16_t qx = (int16_t)((qBuf[3] << 8) | qBuf[2]);
    int16_t qy = (int16_t)((qBuf[5] << 8) | qBuf[4]);
    int16_t qz = (int16_t)((qBuf[7] << 8) | qBuf[6]);

    const float scale = 1.0f / 16384.0f; // 14-bit quaternion representation
    q.w = (float)qw * scale;
    q.x = (float)qx * scale;
    q.y = (float)qy * scale;
    q.z = (float)qz * scale;
    q.normalize();

    // Read 12 bytes for Linear Acceleration (0x28..0x2D) and Gravity (0x2E..0x33)
    uint8_t vBuf[12];
    if (_bus->readBytes(0x28, vBuf, 12)) {
        int16_t lx = (int16_t)((vBuf[1] << 8) | vBuf[0]);
        int16_t ly = (int16_t)((vBuf[3] << 8) | vBuf[2]);
        int16_t lz = (int16_t)((vBuf[5] << 8) | vBuf[4]);

        int16_t gx = (int16_t)((vBuf[7] << 8) | vBuf[6]);
        int16_t gy = (int16_t)((vBuf[9] << 8) | vBuf[8]);
        int16_t gz = (int16_t)((vBuf[11] << 8) | vBuf[10]);

        // 1 LSB = 0.01 m/s^2 (or 1 m/s^2 = 100 LSB)
        linAccel.x = (float)lx / 100.0f;
        linAccel.y = (float)ly / 100.0f;
        linAccel.z = (float)lz / 100.0f;

        gravity.x = (float)gx / 100.0f;
        gravity.y = (float)gy / 100.0f;
        gravity.z = (float)gz / 100.0f;
    }

    return true;
}

bool DriverBNO055::readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) {
    if (!_bus) return false;

    // Read Accel (0x08..0x0D), Mag (0x0E..0x13), Gyro (0x14..0x19) = 18 contiguous bytes!
    uint8_t buffer[18];
    if (!_bus->readBytes(0x08, buffer, 18)) {
        return false;
    }

    int16_t ax = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t ay = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t az = (int16_t)((buffer[5] << 8) | buffer[4]);

    int16_t mx = (int16_t)((buffer[7] << 8) | buffer[6]);
    int16_t my = (int16_t)((buffer[9] << 8) | buffer[8]);
    int16_t mz = (int16_t)((buffer[11] << 8) | buffer[10]);

    int16_t gx = (int16_t)((buffer[13] << 8) | buffer[12]);
    int16_t gy = (int16_t)((buffer[15] << 8) | buffer[14]);
    int16_t gz = (int16_t)((buffer[17] << 8) | buffer[16]);

    // Units: Accel in 1 m/s^2 = 100 LSB
    accel.x = (float)ax / 100.0f;
    accel.y = (float)ay / 100.0f;
    accel.z = (float)az / 100.0f;

    // Mag in 1 uT = 16 LSB
    mag.x = (float)mx / 16.0f;
    mag.y = (float)my / 16.0f;
    mag.z = (float)mz / 16.0f;

    // Gyro in 1 dps = 16 LSB
    gyro.x = (float)gx / 16.0f;
    gyro.y = (float)gy / 16.0f;
    gyro.z = (float)gz / 16.0f;

    // Read temperature (0x34)
    int8_t t = (int8_t)_bus->readByte(0x34);
    temp = (float)t;

    applyCalibration(accel, gyro, mag);
    return true;
}
