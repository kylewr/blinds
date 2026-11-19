#pragma once

#include <cstdint>

struct DriverSettings {
  std::uint16_t runCurrentMa = 600;
  std::uint8_t holdPercent = 50;
  std::uint8_t holdDelay = 8;
  std::uint8_t powerDownDelay = 20;
  std::uint16_t microsteps = 16;
  bool interpolation = true;
  bool doubleEdge = false;
  bool multistepFilter = true;
  bool spreadCycle = false;
  bool shaftInverted = false;
  std::uint32_t stealthThreshold = 0;
  std::uint8_t offTime = 4;
  std::uint8_t blankTime = 24;
  std::int8_t hysteresisEnd = 0;
  std::uint8_t hysteresisStart = 1;
  std::uint8_t pwmOffset = 36;
  std::uint8_t pwmGradient = 14;
  std::uint8_t pwmFrequency = 1;
  bool pwmAutoscale = true;
  bool pwmAutograd = true;
  std::uint8_t freewheel = 0;
  std::uint8_t pwmRegulation = 8;
  std::uint8_t pwmLimit = 12;
  std::uint32_t coolThreshold = 0;
  std::uint8_t stallThreshold = 0;
  std::uint8_t coolMinimum = 0;
  std::uint8_t coolUpStep = 0;
  std::uint8_t coolMaximum = 0;
  std::uint8_t coolDownStep = 0;
  bool coolMinimumQuarter = false;
  std::uint8_t uartReplyDelay = 2;

  static bool validMicrosteps(std::uint16_t value) {
    return value == 1 || value == 2 || value == 4 || value == 8 ||
           value == 16 || value == 32 || value == 64 || value == 128 ||
           value == 256;
  }

  static bool validBlankTime(std::uint8_t value) {
    return value == 16 || value == 24 || value == 36 || value == 54;
  }

  static bool validate(const DriverSettings& value) {
    return value.runCurrentMa >= 100 && value.runCurrentMa <= 2000 &&
           value.holdPercent <= 100 && value.holdDelay <= 15 &&
           validMicrosteps(value.microsteps) && value.stealthThreshold <= 0xFFFFF &&
           value.offTime >= 1 && value.offTime <= 15 &&
           validBlankTime(value.blankTime) && value.hysteresisEnd >= -3 &&
           value.hysteresisEnd <= 12 && value.hysteresisStart >= 1 &&
           value.hysteresisStart <= 8 &&
           value.pwmFrequency <= 3 && value.freewheel <= 3 &&
           value.pwmRegulation <= 15 && value.pwmLimit <= 15 &&
           value.coolThreshold <= 0xFFFFF && value.coolMinimum <= 15 &&
           value.coolUpStep <= 3 && value.coolMaximum <= 15 &&
           value.coolDownStep <= 3 && value.uartReplyDelay <= 15;
  }

  bool changesStepInterpretation(const DriverSettings& other) const {
    return microsteps != other.microsteps || doubleEdge != other.doubleEdge ||
           shaftInverted != other.shaftInverted;
  }
};
