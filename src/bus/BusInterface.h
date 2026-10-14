#ifndef BUS_INTERFACE_H
#define BUS_INTERFACE_H

#include <Arduino.h>
#include "../IMUTypes.h"

class BusInterface {
public:
    virtual ~BusInterface() {}

    virtual bool begin() = 0;
    virtual bool writeByte(uint8_t reg, uint8_t data) = 0;
    virtual bool writeBytes(uint8_t reg, const uint8_t* data, size_t length) = 0;
    virtual uint8_t readByte(uint8_t reg) = 0;
    virtual bool readBytes(uint8_t reg, uint8_t* buffer, size_t length) = 0;
    
    // Modify register bits (read-modify-write)
    virtual bool writeBits(uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data) {
        uint8_t b = readByte(reg);
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1);
        data &= mask;
        b &= ~mask;
        b |= data;
        return writeByte(reg, b);
    }

    virtual BusType getType() const = 0;
    virtual uint8_t getAddressOrCS() const = 0;
    virtual void setAddress(uint8_t addr) = 0;
};

#endif // BUS_INTERFACE_H
