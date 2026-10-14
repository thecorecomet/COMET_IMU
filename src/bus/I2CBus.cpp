#include "I2CBus.h"

I2CBus::I2CBus(TwoWire* wire, uint8_t address) 
    : _wire(wire ? wire : &Wire), _address(address) {}

bool I2CBus::begin() {
    if (!_wire) return false;
    _wire->begin();
    return true;
}

void I2CBus::setClock(uint32_t frequency) {
    if (_wire) {
        _wire->setClock(frequency);
    }
}

bool I2CBus::writeByte(uint8_t reg, uint8_t data) {
    if (!_wire) return false;
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write(data);
    return (_wire->endTransmission() == 0);
}

bool I2CBus::writeBytes(uint8_t reg, const uint8_t* data, size_t length) {
    if (!_wire || !data || length == 0) return false;
    
    _wire->beginTransmission(_address);
    _wire->write(reg);
    for (size_t i = 0; i < length; i++) {
        _wire->write(data[i]);
    }
    return (_wire->endTransmission() == 0);
}

uint8_t I2CBus::readByte(uint8_t reg) {
    uint8_t val = 0;
    if (readBytes(reg, &val, 1)) {
        return val;
    }
    return 0;
}

bool I2CBus::readBytes(uint8_t reg, uint8_t* buffer, size_t length) {
    if (!_wire || !buffer || length == 0) return false;

    // Read in chunks of 32 bytes to ensure compatibility with Arduino Wire buffer limits
    size_t bytesRead = 0;
    while (bytesRead < length) {
        size_t chunk = (length - bytesRead > 32) ? 32 : (length - bytesRead);

        _wire->beginTransmission(_address);
        _wire->write((uint8_t)(reg + bytesRead));
        if (_wire->endTransmission(false) != 0) {
            return false;
        }

        uint8_t count = _wire->requestFrom((uint8_t)_address, (uint8_t)chunk);
        if (count != chunk) {
            return false;
        }

        for (size_t i = 0; i < chunk; i++) {
            if (_wire->available()) {
                buffer[bytesRead++] = (uint8_t)_wire->read();
            } else {
                return false;
            }
        }
    }

    return (bytesRead == length);
}
