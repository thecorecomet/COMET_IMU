#include "DriverBNO08x.h"
#include "../../bus/I2CBus.h"

#define CHANNEL_COMMAND         0
#define CHANNEL_EXECUTABLE      1
#define CHANNEL_CONTROL         2
#define CHANNEL_REPORTS         3

#define SENSOR_REPORTID_ACCELEROMETER       0x01
#define SENSOR_REPORTID_GYROSCOPE_CALIBRATED 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD      0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR     0x05
#define SENSOR_REPORTID_GRAVITY             0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08

DriverBNO08x::DriverBNO08x(BusInterface* bus) : BaseIMUDriver(bus) {
    _lastQuat = Quaternion(1, 0, 0, 0);
    _lastLinAccel = Vector3(0, 0, 0);
    _lastGravity = Vector3(0, 0, 9.80665f);
    _lastRawAccel = Vector3(0, 0, 9.80665f);
    _lastRawGyro = Vector3(0, 0, 0);
    _lastRawMag = Vector3(0, 0, 0);
}

bool DriverBNO08x::sendPacket(uint8_t channel, const uint8_t* payload, uint16_t length) {
    if (!_bus) return false;

    uint16_t packetLength = length + 4;
    uint8_t header[4];
    header[0] = packetLength & 0xFF;
    header[1] = (packetLength >> 8) & 0x7F;
    header[2] = channel;
    header[3] = 0; // sequence

    if (_bus->getType() == BUS_I2C) {
        I2CBus* i2c = static_cast<I2CBus*>(_bus);
        TwoWire* wire = i2c->getWire();
        uint8_t addr = _bus->getAddressOrCS();

        wire->beginTransmission(addr);
        wire->write(header, 4);
        if (payload && length > 0) {
            wire->write(payload, length);
        }
        return (wire->endTransmission() == 0);
    }
    return false;
}

bool DriverBNO08x::receivePacket(uint8_t* buffer, uint16_t bufferSize, uint16_t& packetLength) {
    if (!_bus || !buffer || bufferSize < 4) return false;

    if (_bus->getType() == BUS_I2C) {
        I2CBus* i2c = static_cast<I2CBus*>(_bus);
        TwoWire* wire = i2c->getWire();
        uint8_t addr = _bus->getAddressOrCS();

        // Request 4-byte header first
        uint8_t count = wire->requestFrom(addr, (uint8_t)4);
        if (count < 4) return false;

        uint8_t h0 = wire->read();
        uint8_t h1 = wire->read();
        uint8_t channel = wire->read();
        uint8_t seq = wire->read();

        packetLength = (h1 << 8) | h0;
        packetLength &= ~0x8000; // clear continuation bit

        if (packetLength == 0 || packetLength > bufferSize) {
            return false;
        }

        buffer[0] = h0; buffer[1] = h1; buffer[2] = channel; buffer[3] = seq;

        uint16_t remaining = packetLength - 4;
        uint16_t index = 4;

        while (remaining > 0) {
            uint8_t readLen = (remaining > 32) ? 32 : remaining;
            uint8_t c = wire->requestFrom(addr, readLen);
            for (uint8_t i = 0; i < c; i++) {
                if (wire->available()) {
                    buffer[index++] = wire->read();
                }
            }
            remaining -= c;
            if (c == 0) break;
        }

        return (index == packetLength);
    }
    return false;
}

void DriverBNO08x::enableFeature(uint8_t reportId, uint32_t reportIntervalUs) {
    uint8_t cmd[17];
    cmd[0] = 0xFD; // SET_FEATURE_COMMAND
    cmd[1] = reportId;
    cmd[2] = 0;    // Feature flags
    cmd[3] = 0;    // Change sensitivity LSB
    cmd[4] = 0;    // Change sensitivity MSB
    cmd[5] = (reportIntervalUs >> 0) & 0xFF;
    cmd[6] = (reportIntervalUs >> 8) & 0xFF;
    cmd[7] = (reportIntervalUs >> 16) & 0xFF;
    cmd[8] = (reportIntervalUs >> 24) & 0xFF;
    cmd[9] = 0; cmd[10] = 0; cmd[11] = 0; cmd[12] = 0; // Batch interval
    cmd[13] = 0; cmd[14] = 0; cmd[15] = 0; cmd[16] = 0; // Sensor specific

    sendPacket(CHANNEL_CONTROL, cmd, 17);
}

bool DriverBNO08x::begin(const IMUConfig& config) {
    if (!_bus) return false;
    _bus->begin();
    delay(50);

    // Read startup packets to drain sensor hub init messages
    uint8_t buf[128];
    uint16_t len = 0;
    for (int i = 0; i < 5; i++) {
        receivePacket(buf, sizeof(buf), len);
        delay(10);
    }

    // Enable Rotation Vector report (inbuilt sensor fusion) at desired sample rate
    uint32_t intervalUs = (config.sampleRateHz > 0) ? (1000000UL / config.sampleRateHz) : 10000UL;
    
    // Enable 9-DOF rotation vector or 6-DOF game rotation vector
    if (config.enableMag) {
        enableFeature(SENSOR_REPORTID_ROTATION_VECTOR, intervalUs);
    } else {
        enableFeature(SENSOR_REPORTID_GAME_ROTATION_VECTOR, intervalUs);
    }

    enableFeature(SENSOR_REPORTID_ACCELEROMETER, intervalUs);
    enableFeature(SENSOR_REPORTID_GYROSCOPE_CALIBRATED, intervalUs);
    enableFeature(SENSOR_REPORTID_LINEAR_ACCELERATION, intervalUs);
    enableFeature(SENSOR_REPORTID_GRAVITY, intervalUs);

    return true;
}

void DriverBNO08x::parseReport(const uint8_t* report, uint16_t length) {
    if (length < 9) return;

    // Report format on Channel 3:
    // Byte 0..4: Timestamp
    // Byte 5: Report ID
    // Byte 6: Sequence
    // Byte 7: Status
    // Byte 8..: Payload
    uint8_t reportId = report[5];

    if (reportId == SENSOR_REPORTID_ROTATION_VECTOR || reportId == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
        int16_t rawI = (int16_t)((report[10] << 8) | report[9]);
        int16_t rawJ = (int16_t)((report[12] << 8) | report[11]);
        int16_t rawK = (int16_t)((report[14] << 8) | report[13]);
        int16_t rawReal = (int16_t)((report[16] << 8) | report[15]);

        const float qScale = 1.0f / 16384.0f; // Q14 format
        _lastQuat.x = (float)rawI * qScale;
        _lastQuat.y = (float)rawJ * qScale;
        _lastQuat.z = (float)rawK * qScale;
        _lastQuat.w = (float)rawReal * qScale;
        _lastQuat.normalize();
    } else if (reportId == SENSOR_REPORTID_ACCELEROMETER) {
        int16_t rawX = (int16_t)((report[10] << 8) | report[9]);
        int16_t rawY = (int16_t)((report[12] << 8) | report[11]);
        int16_t rawZ = (int16_t)((report[14] << 8) | report[13]);
        const float aScale = 1.0f / 256.0f; // Q8 format m/s^2
        _lastRawAccel.x = (float)rawX * aScale;
        _lastRawAccel.y = (float)rawY * aScale;
        _lastRawAccel.z = (float)rawZ * aScale;
    } else if (reportId == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
        int16_t rawX = (int16_t)((report[10] << 8) | report[9]);
        int16_t rawY = (int16_t)((report[12] << 8) | report[11]);
        int16_t rawZ = (int16_t)((report[14] << 8) | report[13]);
        const float gScale = (1.0f / 512.0f) * (float)RAD_TO_DEG; // Q9 rad/s -> deg/s
        _lastRawGyro.x = (float)rawX * gScale;
        _lastRawGyro.y = (float)rawY * gScale;
        _lastRawGyro.z = (float)rawZ * gScale;
    } else if (reportId == SENSOR_REPORTID_LINEAR_ACCELERATION) {
        int16_t rawX = (int16_t)((report[10] << 8) | report[9]);
        int16_t rawY = (int16_t)((report[12] << 8) | report[11]);
        int16_t rawZ = (int16_t)((report[14] << 8) | report[13]);
        const float aScale = 1.0f / 256.0f;
        _lastLinAccel.x = (float)rawX * aScale;
        _lastLinAccel.y = (float)rawY * aScale;
        _lastLinAccel.z = (float)rawZ * aScale;
    } else if (reportId == SENSOR_REPORTID_GRAVITY) {
        int16_t rawX = (int16_t)((report[10] << 8) | report[9]);
        int16_t rawY = (int16_t)((report[12] << 8) | report[11]);
        int16_t rawZ = (int16_t)((report[14] << 8) | report[13]);
        const float aScale = 1.0f / 256.0f;
        _lastGravity.x = (float)rawX * aScale;
        _lastGravity.y = (float)rawY * aScale;
        _lastGravity.z = (float)rawZ * aScale;
    }
}

bool DriverBNO08x::readHardwareFusion(Quaternion& q, Vector3& linAccel, Vector3& gravity) {
    uint8_t buffer[128];
    uint16_t length = 0;

    // Check for incoming packets
    if (receivePacket(buffer, sizeof(buffer), length)) {
        if (buffer[2] == CHANNEL_REPORTS) {
            parseReport(buffer + 4, length - 4);
        }
    }

    q = _lastQuat;
    linAccel = _lastLinAccel;
    gravity = _lastGravity;
    return true;
}

bool DriverBNO08x::readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) {
    Quaternion q; Vector3 l, g;
    readHardwareFusion(q, l, g);

    accel = _lastRawAccel;
    gyro = _lastRawGyro;
    mag = _lastRawMag;
    temp = 25.0f; // BNO08x doesn't require ambient temp calibration

    applyCalibration(accel, gyro, mag);
    return true;
}
