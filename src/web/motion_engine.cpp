#include "motion_engine.h"

MotionEngine::MotionEngine(uint8_t stepPin, uint8_t directionPin,
                           uint8_t enablePin, uint8_t ledPin, uint32_t timerHz,
                           uint32_t maximumStepRate,
                           uint32_t pulseHighTicks)
    : planner_(timerHz),
      stepPin_(stepPin),
      directionPin_(directionPin),
      enablePin_(enablePin),
      ledPin_(ledPin),
      timerFrequencyHz_(timerHz),
      maximumStepRate_(maximumStepRate),
      pulseHighTicks_(pulseHighTicks) {}

bool MotionEngine::begin() {
  pinMode(stepPin_, OUTPUT);
  pinMode(directionPin_, OUTPUT);
  pinMode(enablePin_, OUTPUT);
  pinMode(ledPin_, OUTPUT);
  digitalWrite(stepPin_, LOW);
  digitalWrite(directionPin_, LOW);
  digitalWrite(enablePin_, HIGH);
  digitalWrite(ledPin_, HIGH);
  setEnableOutput(false);

  timer_ = timerBegin(timerFrequencyHz_);
  if (timer_ == nullptr) return false;
  timerAttachInterruptArg(timer_, timerThunk, this);
  timerAlarm(timer_, 1, true, 0);
  return true;
}

bool MotionEngine::moveTo(int64_t target, uint32_t speed,
                          uint32_t acceleration) {
  if (!validMotion(speed, acceleration)) return false;
  enable();
  portENTER_CRITICAL(&mux_);
  const bool accepted = planner_.moveTo(target, speed, acceleration);
  portEXIT_CRITICAL(&mux_);
  return accepted;
}

bool MotionEngine::moveBy(int64_t delta, uint32_t speed,
                          uint32_t acceleration) {
  if (!validMotion(speed, acceleration)) return false;
  enable();
  portENTER_CRITICAL(&mux_);
  const bool accepted = planner_.moveBy(delta, speed, acceleration);
  portEXIT_CRITICAL(&mux_);
  return accepted;
}

bool MotionEngine::jog(int8_t direction, uint32_t speed,
                       uint32_t acceleration) {
  if (!validMotion(speed, acceleration)) return false;
  enable();
  portENTER_CRITICAL(&mux_);
  const bool accepted = planner_.jog(direction, speed, acceleration);
  portEXIT_CRITICAL(&mux_);
  return accepted;
}

void MotionEngine::stop() {
  portENTER_CRITICAL(&mux_);
  planner_.stop();
  portEXIT_CRITICAL(&mux_);
}

void MotionEngine::emergencyStop() {
  portENTER_CRITICAL(&mux_);
  planner_.immediateStop();
  pulseHigh_ = false;
  pulseTicksRemaining_ = 0;
  digitalWrite(stepPin_, LOW);
  portEXIT_CRITICAL(&mux_);
  setEnableOutput(false);
  referenced_ = false;
}

bool MotionEngine::zero() {
  portENTER_CRITICAL(&mux_);
  const bool accepted = planner_.zero();
  if (accepted) referenced_ = true;
  portEXIT_CRITICAL(&mux_);
  return accepted;
}

MotionPlanner::Snapshot MotionEngine::snapshot() const {
  portENTER_CRITICAL(&mux_);
  const auto value = planner_.snapshot();
  portEXIT_CRITICAL(&mux_);
  return value;
}

bool MotionEngine::idle() const {
  return snapshot().mode == MotionPlanner::Mode::Idle;
}

void MotionEngine::enable() { setEnableOutput(true); }

void MotionEngine::disable(bool invalidateReference) {
  portENTER_CRITICAL(&mux_);
  planner_.immediateStop();
  pulseHigh_ = false;
  pulseTicksRemaining_ = 0;
  digitalWrite(stepPin_, LOW);
  portEXIT_CRITICAL(&mux_);
  setEnableOutput(false);
  if (invalidateReference) referenced_ = false;
}

bool MotionEngine::enabled() const { return enabled_; }

void MotionEngine::setDirectionInverted(bool inverted) {
  portENTER_CRITICAL(&mux_);
  directionInverted_ = inverted;
  const bool positive = planner_.snapshot().direction > 0;
  digitalWrite(directionPin_, positive != inverted ? HIGH : LOW);
  portEXIT_CRITICAL(&mux_);
}

bool MotionEngine::directionInverted() const { return directionInverted_; }

bool MotionEngine::referenced() const { return referenced_; }

void MotionEngine::invalidateReference() { referenced_ = false; }

void ARDUINO_ISR_ATTR MotionEngine::timerThunk(void* argument) {
  static_cast<MotionEngine*>(argument)->onTimer();
}

void ARDUINO_ISR_ATTR MotionEngine::onTimer() {
  portENTER_CRITICAL_ISR(&mux_);
  if (pulseHigh_) {
    if (pulseTicksRemaining_ > 0) --pulseTicksRemaining_;
    if (pulseTicksRemaining_ == 0) {
      digitalWrite(stepPin_, LOW);
      pulseHigh_ = false;
    }
  }
  const auto tick = planner_.tick();
  if (tick.directionChanged) {
    const bool positive = tick.direction > 0;
    digitalWrite(directionPin_, positive != directionInverted_ ? HIGH : LOW);
  }
  if (tick.step && enabled_) {
    digitalWrite(stepPin_, HIGH);
    pulseHigh_ = true;
    pulseTicksRemaining_ = pulseHighTicks_;
  }
  portEXIT_CRITICAL_ISR(&mux_);
}

bool MotionEngine::validMotion(uint32_t speed, uint32_t acceleration) const {
  return speed > 0 && speed <= maximumStepRate_ && acceleration > 0 &&
         acceleration <= 1000000;
}

void MotionEngine::setEnableOutput(bool enabled) {
  enabled_ = enabled;
  digitalWrite(enablePin_, enabled ? LOW : HIGH);
  digitalWrite(ledPin_, enabled ? LOW : HIGH);
}
