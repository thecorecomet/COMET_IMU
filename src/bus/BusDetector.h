#ifndef BUS_DETECTOR_H
#define BUS_DETECTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "../IMUTypes.h"
#include "I2CBus.h"
#include "SPIBus.h"

struct DetectionResult {
    IMUType type;
    BusType bus;
    uint8_t addressOrCS;
    const char* name;
    bool hasHardwareFusion;
    bool is9DOF;

    DetectionResult() : 
        type(IMU_UNKNOWN), 
        bus(BUS_I2C), 
        addressOrCS(0), 
        name("Unknown"), 
        hasHardwareFusion(false), 
        is9DOF(false) {}
};

class BusDetector {
public:
    static DetectionResult scanI2C(TwoWire* wire = &Wire, uint8_t forcedAddress = 0);
    static DetectionResult scanSPI(uint8_t csPin, SPIClass* spi = &SPI);

    static const char* getIMUName(IMUType type);
    static bool isHardwareFusionSupported(IMUType type);
    static bool is9DOFSupported(IMUType type);

private:
    static IMUType identifyFromRegisters(BusInterface* bus);
};

#endif // BUS_DETECTOR_H
