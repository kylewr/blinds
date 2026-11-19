#include "servo_controller.h"

bool ServoController::attach() {
  if (attached_) return true;
  if (!ledcAttach(pin_, kFrequencyHz, kResolutionBits)) return false;
  attached_ = true;
  writeHardware(pulseUs_);
  return true;
}

void ServoController::detach() {
  sweep_.stop();
  if (attached_) ledcDetach(pin_);
  attached_ = false;
}

bool ServoController::attached() const { return attached_; }

bool ServoController::setRange(uint16_t minimumUs, uint16_t maximumUs) {
  if (minimumUs < 500 || maximumUs > 2500 || minimumUs >= maximumUs) return false;
  minimumUs_ = minimumUs;
  maximumUs_ = maximumUs;
  pulseUs_ = ServoSweep::clampPulse(pulseUs_, minimumUs_, maximumUs_);
  if (attached_) writeHardware(pulseUs_);
  return true;
}

bool ServoController::writeAngle(uint16_t angle) {
  if (angle > ServoSweep::kMaximumAngle) return false;
  return writePulse(ServoSweep::angleToPulse(angle, minimumUs_, maximumUs_));
}

bool ServoController::writePulse(uint16_t pulseUs) {
  sweep_.stop();
  pulseUs_ = ServoSweep::clampPulse(pulseUs, minimumUs_, maximumUs_);
  if (!attached() && !attach()) return false;
  writeHardware(pulseUs_);
  return true;
}

bool ServoController::startSweep(const ServoSweep::Settings& settings,
                                 uint32_t now) {
  if (settings.minimumUs != minimumUs_ || settings.maximumUs != maximumUs_ ||
      !ServoSweep::validate(settings)) return false;
  if (!attached() && !attach()) return false;
  if (!sweep_.start(settings, now)) return false;
  pulseUs_ = settings.startUs;
  writeHardware(pulseUs_);
  return true;
}

void ServoController::stopSweep(bool detachOutput) {
  sweep_.stop();
  if (detachOutput) detach();
}

void ServoController::update(uint32_t now) {
  if (!sweep_.active()) return;
  const uint16_t next = sweep_.update(now);
  if (next != pulseUs_) {
    pulseUs_ = next;
    writeHardware(pulseUs_);
  }
}

uint16_t ServoController::minimumUs() const { return minimumUs_; }
uint16_t ServoController::maximumUs() const { return maximumUs_; }
uint16_t ServoController::pulseUs() const { return pulseUs_; }
uint16_t ServoController::angle() const {
  return static_cast<uint16_t>(
      static_cast<uint32_t>(pulseUs_ - minimumUs_) * ServoSweep::kMaximumAngle /
      (maximumUs_ - minimumUs_));
}
const ServoSweep& ServoController::sweep() const { return sweep_; }

void ServoController::writeHardware(uint16_t pulseUs) {
  const uint32_t maximumDuty = (1UL << kResolutionBits) - 1;
  const uint32_t duty = static_cast<uint64_t>(pulseUs) * maximumDuty / kPeriodUs;
  ledcWrite(pin_, duty);
}
