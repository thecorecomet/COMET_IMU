#include "SPIBus.h"

SPIBus::SPIBus(uint8_t csPin, SPIClass* spi, uint32_t clockFreq)
    : _csPin(csPin), 
      _spi(spi ? spi : &SPI), 
      _spiSettings(clockFreq, MSBFIRST, SPI_MODE0),
      _readMask(0x80), 
      _writeMask(0x7F) {}

bool SPIBus::begin() {
    if (!_spi) return false;
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    _spi->begin();
    return true;
}

void SPIBus::setClock(uint32_t frequency) {
    _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

bool SPIBus::writeByte(uint8_t reg, uint8_t data) {
    if (!_spi) return false;
    
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    
    _spi->transfer(reg & _writeMask);
    _spi->transfer(data);
    
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
    return true;
}

bool SPIBus::writeBytes(uint8_t reg, const uint8_t* data, size_t length) {
    if (!_spi || !data || length == 0) return false;

    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);

    _spi->transfer(reg & _writeMask);
    for (size_t i = 0; i < length; i++) {
        _spi->transfer(data[i]);
    }

    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
    return true;
}

uint8_t SPIBus::readByte(uint8_t reg) {
    uint8_t val = 0;
    readBytes(reg, &val, 1);
    return val;
}

bool SPIBus::readBytes(uint8_t reg, uint8_t* buffer, size_t length) {
    if (!_spi || !buffer || length == 0) return false;

    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);

    _spi->transfer(reg | _readMask);
    for (size_t i = 0; i < length; i++) {
        buffer[i] = _spi->transfer(0x00);
    }

    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
    return true;
}
