#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <Wire.h>
#include "BusInterface.h"

class I2CBus : public BusInterface {
public:
    I2CBus(TwoWire* wire = &Wire, uint8_t address = 0x68);
    virtual ~I2CBus() {}

    bool begin() override;
    bool writeByte(uint8_t reg, uint8_t data) override;
    bool writeBytes(uint8_t reg, const uint8_t* data, size_t length) override;
    uint8_t readByte(uint8_t reg) override;
    bool readBytes(uint8_t reg, uint8_t* buffer, size_t length) override;

    BusType getType() const override { return BUS_I2C; }
    uint8_t getAddressOrCS() const override { return _address; }
    void setAddress(uint8_t addr) override { _address = addr; }

    void setClock(uint32_t frequency);
    TwoWire* getWire() const { return _wire; }

private:
    TwoWire* _wire;
    uint8_t _address;
};

#endif // I2C_BUS_H
