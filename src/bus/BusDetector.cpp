#include "BusDetector.h"

const char* BusDetector::getIMUName(IMUType type) {
    switch (type) {
        case IMU_MPU6050:  return "InvenSense MPU-6050 (6-DOF)";
        case IMU_MPU6500:  return "InvenSense MPU-6500 (6-DOF)";
        case IMU_MPU9250:  return "InvenSense MPU-9250 (9-DOF)";
        case IMU_ICM20948: return "TDK InvenSense ICM-20948 (9-DOF)";
        case IMU_ICM42688: return "TDK InvenSense ICM-42688 (6-DOF)";
        case IMU_BNO055:   return "Bosch BNO055 (Absolute Orientation 9-DOF)";
        case IMU_BNO080:   return "Bosch Sensortec BNO080 (SH-2 Hub 9-DOF)";
        case IMU_BNO085:   return "Bosch Sensortec BNO085 (SH-2 Hub 9-DOF)";
        case IMU_BMI160:   return "Bosch BMI160 (6-DOF)";
        case IMU_BMI270:   return "Bosch BMI270 (6-DOF)";
        case IMU_LSM6DSOX: return "STMicroelectronics LSM6DSOX (6-DOF)";
        case IMU_LSM6DS3:  return "STMicroelectronics LSM6DS3 (6-DOF)";
        case IMU_LSM9DS1:  return "STMicroelectronics LSM9DS1 (9-DOF)";
        default:           return "Unknown / Not Detected";
    }
}

bool BusDetector::isHardwareFusionSupported(IMUType type) {
    switch (type) {
        case IMU_BNO055:
        case IMU_BNO080:
        case IMU_BNO085:
        case IMU_MPU6050:
        case IMU_MPU9250:
        case IMU_ICM20948:
            return true;
        default:
            return false;
    }
}

bool BusDetector::is9DOFSupported(IMUType type) {
    switch (type) {
        case IMU_BNO055:
        case IMU_BNO080:
        case IMU_BNO085:
        case IMU_MPU9250:
        case IMU_ICM20948:
        case IMU_LSM9DS1:
            return true;
        default:
            return false;
    }
}

IMUType BusDetector::identifyFromRegisters(BusInterface* bus) {
    if (!bus) return IMU_UNKNOWN;

    // 1. Check register 0x00 (Used by ICM-20948 Bank 0, BNO055, BMI160, BMI270)
    uint8_t id00 = bus->readByte(0x00);
    if (id00 == 0xA0) return IMU_BNO055;
    if (id00 == 0xEA) return IMU_ICM20948;
    if (id00 == 0xD1) return IMU_BMI160;
    if (id00 == 0x24) return IMU_BMI270;

    // 2. Check register 0x75 (WHO_AM_I for InvenSense MPU & ICM-42688)
    uint8_t id75 = bus->readByte(0x75);
    if (id75 == 0x47) return IMU_ICM42688;
    if (id75 == 0x71 || id75 == 0x73) return IMU_MPU9250;
    if (id75 == 0x70) return IMU_MPU6500;
    if (id75 == 0x68 || id75 == 0x69 || id75 == 0x72) return IMU_MPU6050;

    // 3. Check register 0x0F (WHO_AM_I for ST LSM family)
    uint8_t id0F = bus->readByte(0x0F);
    if (id0F == 0x6C) return IMU_LSM6DSOX;
    if (id0F == 0x69) return IMU_LSM6DS3;
    if (id0F == 0x68) return IMU_LSM9DS1;

    return IMU_UNKNOWN;
}

DetectionResult BusDetector::scanI2C(TwoWire* wire, uint8_t forcedAddress) {
    DetectionResult result;
    result.bus = BUS_I2C;
    if (!wire) return result;

    wire->begin();

    // Candidate addresses for popular IMUs
    const uint8_t candidateAddresses[] = {
        0x68, 0x69, // MPU6050, MPU6500, MPU9250, ICM20948, ICM42688, BMI160, BMI270
        0x28, 0x29, // BNO055
        0x4A, 0x4B, // BNO080 / BNO085
        0x6A, 0x6B  // LSM6DSOX, LSM6DS3, LSM9DS1
    };

    size_t count = sizeof(candidateAddresses) / sizeof(candidateAddresses[0]);

    for (size_t i = 0; i < count; i++) {
        uint8_t addr = candidateAddresses[i];
        if (forcedAddress != 0 && addr != forcedAddress) {
            continue;
        }

        // Check if device acknowledges on bus
        wire->beginTransmission(addr);
        uint8_t error = wire->endTransmission();

        if (error == 0) {
            // Check for BNO08x at 0x4A or 0x4B
            if (addr == 0x4A || addr == 0x4B) {
                result.type = IMU_BNO085;
                result.addressOrCS = addr;
                result.name = getIMUName(result.type);
                result.hasHardwareFusion = true;
                result.is9DOF = true;
                return result;
            }

            I2CBus bus(wire, addr);
            IMUType detected = identifyFromRegisters(&bus);

            if (detected != IMU_UNKNOWN) {
                result.type = detected;
                result.addressOrCS = addr;
                result.name = getIMUName(detected);
                result.hasHardwareFusion = isHardwareFusionSupported(detected);
                result.is9DOF = is9DOFSupported(detected);
                return result;
            }
        }
    }

    return result;
}

DetectionResult BusDetector::scanSPI(uint8_t csPin, SPIClass* spi) {
    DetectionResult result;
    result.bus = BUS_SPI;
    result.addressOrCS = csPin;

    if (!spi) return result;

    SPIBus bus(csPin, spi);
    bus.begin();

    IMUType detected = identifyFromRegisters(&bus);

    if (detected != IMU_UNKNOWN) {
        result.type = detected;
        result.name = getIMUName(detected);
        result.hasHardwareFusion = isHardwareFusionSupported(detected);
        result.is9DOF = is9DOFSupported(detected);
    }

    return result;
}
