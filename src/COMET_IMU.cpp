#include "COMET_IMU.h"

// Driver implementations
#include "drivers/InvenSense/DriverMPU6050.h"
#include "drivers/InvenSense/DriverMPU9250.h"
#include "drivers/InvenSense/DriverICM20948.h"
#include "drivers/InvenSense/DriverICM42688.h"
#include "drivers/Bosch/DriverBNO055.h"
#include "drivers/Bosch/DriverBNO08x.h"
#include "drivers/Bosch/DriverBMI160.h"
#include "drivers/ST/DriverLSM6DSOX.h"
#include "drivers/ST/DriverLSM9DS1.h"

COMET_IMU::COMET_IMU() 
    : _driver(nullptr),
      _bus(nullptr),
      _fusion(nullptr),
      _detectedType(IMU_UNKNOWN),
      _lastUpdateMicros(0),
      _isInitialized(false),
      _ownsBus(false),
      _ownsFusion(false) {}

COMET_IMU::~COMET_IMU() {
    cleanup();
}

void COMET_IMU::cleanup() {
    if (_driver) {
        delete _driver;
        _driver = nullptr;
    }
    if (_ownsBus && _bus) {
        delete _bus;
        _bus = nullptr;
        _ownsBus = false;
    }
    if (_ownsFusion && _fusion) {
        delete _fusion;
        _fusion = nullptr;
        _ownsFusion = false;
    }
    _isInitialized = false;
}

bool COMET_IMU::setupDriver(IMUType type) {
    if (!_bus) return false;

    if (_driver) {
        delete _driver;
        _driver = nullptr;
    }

    _detectedType = type;

    switch (type) {
        case IMU_MPU6050:
            _driver = new DriverMPU6050(_bus);
            break;
        case IMU_MPU6500:
        case IMU_MPU9250:
            _driver = new DriverMPU9250(_bus);
            break;
        case IMU_ICM20948:
            _driver = new DriverICM20948(_bus);
            break;
        case IMU_ICM42688:
            _driver = new DriverICM42688(_bus);
            break;
        case IMU_BNO055:
            _driver = new DriverBNO055(_bus);
            break;
        case IMU_BNO080:
        case IMU_BNO085:
            _driver = new DriverBNO08x(_bus);
            break;
        case IMU_BMI160:
        case IMU_BMI270:
            _driver = new DriverBMI160(_bus);
            break;
        case IMU_LSM6DSOX:
        case IMU_LSM6DS3:
            _driver = new DriverLSM6DSOX(_bus);
            break;
        case IMU_LSM9DS1:
            _driver = new DriverLSM9DS1(_bus);
            break;
        default:
            return false;
    }

    if (!_driver) return false;

    // Initialize physical driver
    if (!_driver->begin(_config)) {
        delete _driver;
        _driver = nullptr;
        return false;
    }

    // Configure sensor fusion
    setFusionMode(_config.fusionMode);

    _isInitialized = true;
    _lastUpdateMicros = micros();
    return true;
}

bool COMET_IMU::autoDetect(TwoWire* wire, const IMUConfig& config) {
    cleanup();
    _config = config;

    DetectionResult res = BusDetector::scanI2C(wire);
    if (res.type == IMU_UNKNOWN) {
        return false;
    }

    _bus = new I2CBus(wire, res.addressOrCS);
    _ownsBus = true;

    return setupDriver(res.type);
}

bool COMET_IMU::autoDetectSPI(uint8_t csPin, SPIClass* spi, const IMUConfig& config) {
    cleanup();
    _config = config;

    DetectionResult res = BusDetector::scanSPI(csPin, spi);
    if (res.type == IMU_UNKNOWN) {
        return false;
    }

    _bus = new SPIBus(csPin, spi);
    _ownsBus = true;

    return setupDriver(res.type);
}

bool COMET_IMU::begin(IMUType type, TwoWire* wire, const IMUConfig& config) {
    uint8_t defaultAddr = 0x68;
    if (type == IMU_BNO055) defaultAddr = 0x28;
    else if (type == IMU_BNO080 || type == IMU_BNO085) defaultAddr = 0x4A;
    else if (type == IMU_LSM6DSOX || type == IMU_LSM6DS3 || type == IMU_LSM9DS1) defaultAddr = 0x6A;

    return beginI2C(type, defaultAddr, wire, config);
}

bool COMET_IMU::beginI2C(IMUType type, uint8_t address, TwoWire* wire, const IMUConfig& config) {
    cleanup();
    _config = config;

    _bus = new I2CBus(wire, address);
    _ownsBus = true;

    return setupDriver(type);
}

bool COMET_IMU::beginSPI(IMUType type, uint8_t csPin, SPIClass* spi, const IMUConfig& config) {
    cleanup();
    _config = config;

    _bus = new SPIBus(csPin, spi);
    _ownsBus = true;

    return setupDriver(type);
}

void COMET_IMU::setFusionMode(FusionMode mode) {
    _config.fusionMode = mode;

    if (_ownsFusion && _fusion) {
        delete _fusion;
        _fusion = nullptr;
        _ownsFusion = false;
    }

    if (mode == FUSION_AUTO) {
        // If the sensor has built-in hardware fusion, prioritize it to eliminate CPU load
        if (_driver && _driver->hasHardwareFusion()) {
            return;
        }
        // Otherwise fallback to Madgwick AHRS filter
        _fusion = new MadgwickFilter(0.1f);
        _ownsFusion = true;
    } else if (mode == FUSION_MADGWICK) {
        _fusion = new MadgwickFilter(0.1f);
        _ownsFusion = true;
    } else if (mode == FUSION_MAHONY) {
        _fusion = new MahonyFilter(2.0f, 0.005f);
        _ownsFusion = true;
    }
}

bool COMET_IMU::update() {
    if (!_isInitialized || !_driver) return false;

    uint32_t currentMicros = micros();
    float dt = (float)(currentMicros - _lastUpdateMicros) * 1e-6f;
    _lastUpdateMicros = currentMicros;

    // Guard against negative or absurd dt on counter rollover
    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.01f;
    }

    _data.timestampMicros = currentMicros;
    _data.hasMag = _driver->is9DOF();

    // Check if on-chip hardware fusion should be used
    bool useHardwareFusion = (_config.fusionMode == FUSION_HARDWARE) ||
                             (_config.fusionMode == FUSION_AUTO && _driver->hasHardwareFusion());

    if (useHardwareFusion && _driver->hasHardwareFusion()) {
        _driver->readHardwareFusion(_data.orientation, _data.linearAccel, _data.gravity);
        _data.euler = _data.orientation.toEuler();
        _data.hasHardwareFusion = true;

        // Also read raw readings and temperature
        _driver->readRaw(_data.accel, _data.gyro, _data.mag, _data.temperature);
        return true;
    }

    // Read raw sensor vectors
    if (!_driver->readRaw(_data.accel, _data.gyro, _data.mag, _data.temperature)) {
        return false;
    }

    _data.hasHardwareFusion = false;

    // Run software fusion filter if enabled
    if (_fusion && _config.fusionMode != FUSION_OFF) {
        if (_driver->is9DOF() && _config.enableMag) {
            _fusion->update9DOF(
                _data.gyro.x, _data.gyro.y, _data.gyro.z,
                _data.accel.x, _data.accel.y, _data.accel.z,
                _data.mag.x, _data.mag.y, _data.mag.z,
                dt
            );
        } else {
            _fusion->update6DOF(
                _data.gyro.x, _data.gyro.y, _data.gyro.z,
                _data.accel.x, _data.accel.y, _data.accel.z,
                dt
            );
        }

        _data.orientation = _fusion->getQuaternion();
        _data.euler = _data.orientation.toEuler();
        _data.gravity = _fusion->getGravity();
        _data.linearAccel = _fusion->getLinearAccel(_data.accel);
    }

    return true;
}

void COMET_IMU::calibrateGyro(uint16_t samples, uint16_t delayMs) {
    if (_driver) {
        _driver->calibrateGyro(samples, delayMs);
    }
}

void COMET_IMU::calibrateAccel(uint16_t samples, uint16_t delayMs) {
    if (_driver) {
        _driver->calibrateAccel(samples, delayMs);
    }
}

const char* COMET_IMU::getSensorName() const {
    if (_driver) {
        return _driver->getName();
    }
    return BusDetector::getIMUName(_detectedType);
}
