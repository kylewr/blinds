#pragma once

#include "motion_planner.h"

#include <Arduino.h>

class MotionEngine {
 public:
  MotionEngine(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin,
               uint8_t ledPin, uint32_t timerHz, uint32_t maximumStepRate,
               uint32_t pulseHighTicks);

  bool begin();
  bool moveTo(int64_t target, uint32_t speed, uint32_t acceleration);
  bool moveBy(int64_t delta, uint32_t speed, uint32_t acceleration);
  bool jog(int8_t direction, uint32_t speed, uint32_t acceleration);
  void stop();
  void emergencyStop();
  bool zero();
  MotionPlanner::Snapshot snapshot() const;
  bool idle() const;

  void enable();
  void disable(bool invalidateReference = true);
  bool enabled() const;
  void setDirectionInverted(bool inverted);
  bool directionInverted() const;
  bool referenced() const;
  void invalidateReference();

 private:
  static void ARDUINO_ISR_ATTR timerThunk(void* argument);
  void ARDUINO_ISR_ATTR onTimer();
  bool validMotion(uint32_t speed, uint32_t acceleration) const;
  void setEnableOutput(bool enabled);

  mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  MotionPlanner planner_;
  const uint8_t stepPin_;
  const uint8_t directionPin_;
  const uint8_t enablePin_;
  const uint8_t ledPin_;
  const uint32_t timerFrequencyHz_;
  const uint32_t maximumStepRate_;
  const uint32_t pulseHighTicks_;
  hw_timer_t* timer_ = nullptr;
  volatile bool enabled_ = false;
  volatile bool pulseHigh_ = false;
  volatile uint32_t pulseTicksRemaining_ = 0;
  volatile bool directionInverted_ = false;
  volatile bool referenced_ = false;
};
