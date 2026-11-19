#pragma once

#include "driver_settings.h"
#include "failure_latch.h"

#include <Arduino.h>
#include <TMCStepper.h>

struct TmcDiagnostics {
  bool connected = false;
  bool faultLatched = false;
  bool uartEcho = false;
  bool uartRxIdleHigh = false;
  int8_t detectedAddress = -1;
  uint32_t detectedBaud = 0;
  uint8_t consecutiveFailures = 0;
  uint8_t connectionCode = 0;
  uint8_t interfaceCount = 0;
  uint8_t version = 0;
  uint32_t inputStatus = 0;
  uint8_t globalStatus = 0;
  uint32_t driverStatus = 0;
  uint32_t tstep = 0;
  uint16_t microstepCounter = 0;
  int16_t coilCurrentA = 0;
  int16_t coilCurrentB = 0;
  uint16_t stallResult = 0;
  uint8_t pwmScaleSum = 0;
  int16_t pwmScaleAuto = 0;
  uint8_t pwmOffsetAuto = 0;
  uint8_t pwmGradientAuto = 0;
  uint32_t gconf = 0;
  uint16_t slaveConfig = 0;
  uint32_t otpRead = 0;
  uint16_t factoryConfig = 0;
  uint32_t currentConfig = 0;
  uint8_t powerDown = 0;
  uint32_t actualVelocity = 0;
  uint32_t stealthThreshold = 0;
  uint32_t coolThreshold = 0;
  uint8_t stallThreshold = 0;
  uint16_t coolConfig = 0;
  uint32_t microstepCurrents = 0;
  uint32_t chopperConfig = 0;
  uint32_t pwmConfig = 0;
  uint32_t pwmScale = 0;
  uint32_t pwmAuto = 0;
};

class Tmc2209Uart {
 public:
  Tmc2209Uart(HardwareSerial& serial, uint8_t receivePin, uint8_t transmitPin,
              uint16_t rmsCurrentMilliamps, uint16_t microsteps);

  void begin();
  bool retry();
  bool applySettings(const DriverSettings& settings);
  bool pollDiagnostics();
  void clearStatus();
  const DriverSettings& settings() const;
  const TmcDiagnostics& diagnostics() const;
  bool healthy() const;
  bool faultLatched() const;

 private:
  static constexpr uint32_t kBaudRate = 115200;
  static constexpr float kSenseResistorOhms = 0.11F;
  static constexpr uint8_t kDriverAddress = 0;
  static constexpr uint8_t kFailureLimit = 3;

  HardwareSerial& serial_;
  TMC2209Stepper driver_;
  const uint8_t receivePin_;
  const uint8_t transmitPin_;
  DriverSettings settings_{};
  TmcDiagnostics diagnostics_{};
  ConsecutiveFailureLatch failures_{kFailureLimit};

  bool probeUartEcho();
  int8_t scanAddresses(uint32_t& detectedBaud);
  void recordFailure(uint8_t connectionCode);
  void readDiagnosticRegisters();
};
