#pragma once

#include "servo_sweep.h"

#include <Arduino.h>

class ServoController {
 public:
  explicit ServoController(uint8_t pin) : pin_(pin) {}

  bool attach();
  void detach();
  bool attached() const;
  bool setRange(uint16_t minimumUs, uint16_t maximumUs);
  bool writeAngle(uint16_t angle);
  bool writePulse(uint16_t pulseUs);
  bool startSweep(const ServoSweep::Settings& settings, uint32_t now);
  void stopSweep(bool detachOutput);
  void update(uint32_t now);

  uint16_t minimumUs() const;
  uint16_t maximumUs() const;
  uint16_t pulseUs() const;
  uint16_t angle() const;
  const ServoSweep& sweep() const;

 private:
  static constexpr uint32_t kFrequencyHz = 100;
  static constexpr uint8_t kResolutionBits = 14;
  static constexpr uint32_t kPeriodUs = 10000;

  const uint8_t pin_;
  uint16_t minimumUs_ = 500;
  uint16_t maximumUs_ = 2500;
  uint16_t pulseUs_ = 1500;
  bool attached_ = false;
  ServoSweep sweep_;

  void writeHardware(uint16_t pulseUs);
};
