#ifndef SPI_BUS_H
#define SPI_BUS_H

#include <SPI.h>
#include "BusInterface.h"

class SPIBus : public BusInterface {
public:
    SPIBus(uint8_t csPin, SPIClass* spi = &SPI, uint32_t clockFreq = 8000000);
    virtual ~SPIBus() {}

    bool begin() override;
    bool writeByte(uint8_t reg, uint8_t data) override;
    bool writeBytes(uint8_t reg, const uint8_t* data, size_t length) override;
    uint8_t readByte(uint8_t reg) override;
    bool readBytes(uint8_t reg, uint8_t* buffer, size_t length) override;

    BusType getType() const override { return BUS_SPI; }
    uint8_t getAddressOrCS() const override { return _csPin; }
    void setAddress(uint8_t csPin) override { _csPin = csPin; }

    void setReadMask(uint8_t mask) { _readMask = mask; }
    void setWriteMask(uint8_t mask) { _writeMask = mask; }
    void setClock(uint32_t frequency);

    SPIClass* getSPI() const { return _spi; }

private:
    uint8_t _csPin;
    SPIClass* _spi;
    SPISettings _spiSettings;
    uint8_t _readMask;
    uint8_t _writeMask;
};

#endif // SPI_BUS_H
