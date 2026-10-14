#ifndef DRIVER_BNO08X_H
#define DRIVER_BNO08X_H

#include "../BaseIMUDriver.h"

class DriverBNO08x : public BaseIMUDriver {
public:
    DriverBNO08x(BusInterface* bus);

    bool begin(const IMUConfig& config) override;
    bool readRaw(Vector3& accel, Vector3& gyro, Vector3& mag, float& temp) override;
    
    // BNO080/BNO085 has on-chip SH-2 sensor hub fusion!
    bool hasHardwareFusion() const override { return true; }
    bool readHardwareFusion(Quaternion& q, Vector3& linAccel, Vector3& gravity) override;

    bool is9DOF() const override { return true; }
    IMUType getType() const override { return IMU_BNO085; }
    const char* getName() const override { return "Bosch Sensortec BNO085 (SH-2 Hub)"; }

    void enableFeature(uint8_t reportId, uint32_t reportIntervalUs);

private:
    Quaternion _lastQuat;
    Vector3 _lastLinAccel;
    Vector3 _lastGravity;
    Vector3 _lastRawAccel;
    Vector3 _lastRawGyro;
    Vector3 _lastRawMag;

    bool receivePacket(uint8_t* buffer, uint16_t bufferSize, uint16_t& packetLength);
    bool sendPacket(uint8_t channel, const uint8_t* payload, uint16_t length);
    void parseReport(const uint8_t* report, uint16_t length);
};

#endif // DRIVER_BNO08X_H
