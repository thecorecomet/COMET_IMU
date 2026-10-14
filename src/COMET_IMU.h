#ifndef COMET_IMU_H
#define COMET_IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "IMUTypes.h"
#include "bus/BusInterface.h"
#include "bus/I2CBus.h"
#include "bus/SPIBus.h"
#include "bus/BusDetector.h"
#include "fusion/SensorFusion.h"
#include "fusion/MadgwickFilter.h"
#include "fusion/MahonyFilter.h"
#include "drivers/BaseIMUDriver.h"

class COMET_IMU {
public:
    COMET_IMU();
    ~COMET_IMU();

    // ========================================================
    // 1. AUTO-DETECTION FRONT-END
    // ========================================================
    
    // Auto-detect any supported IMU on I2C bus (Wire or Wire1)
    bool autoDetect(TwoWire* wire = &Wire, const IMUConfig& config = IMUConfig());

    // Auto-detect any supported IMU on SPI bus with Chip Select pin
    bool autoDetectSPI(uint8_t csPin, SPIClass* spi = &SPI, const IMUConfig& config = IMUConfig());

    // ========================================================
    // 2. EXPLICIT SELECTION FRONT-END
    // ========================================================

    // Quick start with specific IMU model (auto-selects standard I2C address)
    bool begin(IMUType type, TwoWire* wire = &Wire, const IMUConfig& config = IMUConfig());

    // Explicit I2C initialization with specific address
    bool beginI2C(IMUType type, uint8_t address, TwoWire* wire = &Wire, const IMUConfig& config = IMUConfig());

    // Explicit SPI initialization with specific CS pin
    bool beginSPI(IMUType type, uint8_t csPin, SPIClass* spi = &SPI, const IMUConfig& config = IMUConfig());

    // ========================================================
    // 3. MAIN SENSOR UPDATE LOOP
    // ========================================================
    
    // Updates sensor readings and recalculates sensor fusion orientation
    // Call this inside your void loop()
    bool update();

    // ========================================================
    // 4. UNIFIED ORIENTATION & MOTION GETTERS
    // ========================================================
    
    // Orientation as unit Quaternion (W, X, Y, Z)
    Quaternion getQuaternion() const { return _data.orientation; }

    // Orientation as Euler angles (Roll, Pitch, Yaw in degrees)
    EulerAngles getEuler() const { return _data.euler; }

    // Accelerometer vector in m/s^2
    Vector3 getAccel() const { return _data.accel; }

    // Gyroscope vector in degrees/sec
    Vector3 getGyro() const { return _data.gyro; }

    // Magnetometer vector in microTesla (uT)
    Vector3 getMag() const { return _data.mag; }

    // Linear Acceleration (gravity vector subtracted from raw acceleration)
    Vector3 getLinearAccel() const { return _data.linearAccel; }

    // Estimated Gravity direction vector in sensor frame
    Vector3 getGravity() const { return _data.gravity; }

    // Die temperature in Celsius
    float getTemperature() const { return _data.temperature; }

    // Full snapshot of all sensor data
    const SensorData& getData() const { return _data; }

    // ========================================================
    // 5. CALIBRATION FRONT-END
    // ========================================================
    
    // Zero-rate gyroscope bias calibration while stationary
    void calibrateGyro(uint16_t samples = 300, uint16_t delayMs = 4);

    // Accelerometer calibration assuming sensor is flat with Z-axis pointing up
    void calibrateAccel(uint16_t samples = 300, uint16_t delayMs = 4);

    // ========================================================
    // 6. SENSOR INFORMATION & STATUS
    // ========================================================
    
    const char* getSensorName() const;
    IMUType getDetectedType() const { return _detectedType; }
    BusType getBusType() const { return _bus ? _bus->getType() : BUS_I2C; }
    bool is9DOF() const { return _driver ? _driver->is9DOF() : false; }
    bool hasHardwareFusion() const { return _driver ? _driver->hasHardwareFusion() : false; }
    bool isInitialized() const { return _isInitialized; }

    // Sensor Fusion Mode configuration
    void setFusionMode(FusionMode mode);
    FusionMode getFusionMode() const { return _config.fusionMode; }

    // Access to low-level driver or fusion engine if needed
    BaseIMUDriver* getDriver() const { return _driver; }
    SensorFusion* getFusionEngine() const { return _fusion; }

private:
    BaseIMUDriver* _driver;
    BusInterface*  _bus;
    SensorFusion*  _fusion;
    IMUType        _detectedType;
    IMUConfig      _config;
    SensorData     _data;
    uint32_t       _lastUpdateMicros;
    bool           _isInitialized;
    bool           _ownsBus;
    bool           _ownsFusion;

    bool setupDriver(IMUType type);
    void cleanup();
};

#endif // COMET_IMU_H
