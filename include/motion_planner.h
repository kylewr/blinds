#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>

class MotionPlanner {
 public:
  static constexpr std::uint32_t kRateScale = 1U << 16;

  enum class Mode : std::uint8_t { Idle, Finite, Continuous, Stopping };
  enum class Phase : std::uint8_t { Stopped, Accelerating, Running, Decelerating };

  struct Snapshot {
    Mode mode = Mode::Idle;
    Phase phase = Phase::Stopped;
    std::int64_t position = 0;
    std::int64_t target = 0;
    std::uint32_t speed = 0;
    std::uint32_t commandSpeed = 0;
    std::uint32_t acceleration = 0;
    std::uint64_t remaining = 0;
    std::uint64_t moveTotal = 0;
    std::int8_t direction = 1;
  };

  struct TickResult {
    bool step = false;
    bool directionChanged = false;
    std::int8_t direction = 1;
  };

  explicit MotionPlanner(std::uint32_t tickRateHz = 50000)
      : tickRateHz_(tickRateHz) {}

  bool moveTo(std::int64_t target, std::uint32_t speed,
              std::uint32_t acceleration) {
    if (!validCommand(speed, acceleration)) return false;
    pendingJog_ = false;
    target_ = target;
    moveStart_ = position_;
    commandSpeed_ = speed;
    acceleration_ = acceleration;
    if (position_ == target && speedQ_ == 0) {
      mode_ = Mode::Idle;
      phase_ = Phase::Stopped;
      return true;
    }
    mode_ = Mode::Finite;
    return true;
  }

  bool moveBy(std::int64_t delta, std::uint32_t speed,
              std::uint32_t acceleration) {
    if ((delta > 0 && position_ > std::numeric_limits<std::int64_t>::max() - delta) ||
        (delta < 0 && position_ < std::numeric_limits<std::int64_t>::min() - delta)) {
      return false;
    }
    return moveTo(position_ + delta, speed, acceleration);
  }

  bool jog(std::int8_t direction, std::uint32_t speed,
           std::uint32_t acceleration) {
    if (!validCommand(speed, acceleration) || direction == 0) return false;
    const std::int8_t requested = direction > 0 ? 1 : -1;
    if (pendingJog_) {
      pendingDirection_ = requested;
      pendingSpeed_ = speed;
      pendingAcceleration_ = acceleration;
      return true;
    }
    if (speedQ_ > 0 && acceleration != acceleration_) {
      pendingJog_ = true;
      pendingDirection_ = requested;
      pendingSpeed_ = speed;
      pendingAcceleration_ = acceleration;
      mode_ = Mode::Stopping;
      return true;
    }
    requestedDirection_ = requested;
    commandSpeed_ = speed;
    acceleration_ = acceleration;
    mode_ = Mode::Continuous;
    return true;
  }

  bool stop() {
    pendingJog_ = false;
    if (mode_ != Mode::Idle) mode_ = Mode::Stopping;
    return true;
  }

  void immediateStop() {
    mode_ = Mode::Idle;
    phase_ = Phase::Stopped;
    speedQ_ = 0;
    phaseAccumulator_ = 0;
    target_ = position_;
    pendingJog_ = false;
  }

  bool zero() {
    if (mode_ != Mode::Idle) return false;
    position_ = 0;
    target_ = 0;
    moveStart_ = 0;
    return true;
  }

  TickResult tick() {
    TickResult result;
    result.direction = direction_;
    if (mode_ == Mode::Idle) {
      phase_ = Phase::Stopped;
      return result;
    }

    std::int8_t desiredDirection = direction_;
    std::uint32_t desiredSpeed = 0;
    if (mode_ == Mode::Finite) {
      if (target_ == position_) {
        immediateStop();
        return result;
      }
      desiredDirection = target_ > position_ ? 1 : -1;
      const std::uint64_t remaining = distance(position_, target_);
      const std::uint64_t twiceAcceleration = 2ULL * acceleration_;
      const std::uint64_t brakingRateSquared =
          remaining > std::numeric_limits<std::uint64_t>::max() / twiceAcceleration
              ? std::numeric_limits<std::uint64_t>::max()
              : twiceAcceleration * remaining;
      desiredSpeed = static_cast<std::uint32_t>(integerSqrt(brakingRateSquared));
      desiredSpeed = std::max<std::uint32_t>(1, std::min(commandSpeed_, desiredSpeed));
    } else if (mode_ == Mode::Continuous) {
      desiredDirection = requestedDirection_;
      desiredSpeed = commandSpeed_;
    }

    const std::uint32_t increment = accelerationIncrement(acceleration_);
    if (desiredDirection != direction_ && speedQ_ > 0) {
      speedQ_ = subtractFloor(speedQ_, increment);
      phase_ = Phase::Decelerating;
    } else {
      if (desiredDirection != direction_) {
        direction_ = desiredDirection;
        directionSettleTicks_ = 1;
        phaseAccumulator_ = 0;
        result.directionChanged = true;
        result.direction = direction_;
      }
      const std::uint64_t desiredQ =
          static_cast<std::uint64_t>(desiredSpeed) * kRateScale;
      if (speedQ_ < desiredQ) {
        speedQ_ = std::min(desiredQ, speedQ_ + increment);
        phase_ = Phase::Accelerating;
      } else if (speedQ_ > desiredQ) {
        speedQ_ = subtractFloor(speedQ_, increment);
        if (speedQ_ < desiredQ) speedQ_ = desiredQ;
        phase_ = Phase::Decelerating;
      } else {
        phase_ = desiredSpeed == 0 ? Phase::Stopped : Phase::Running;
      }
    }

    if (mode_ == Mode::Stopping && speedQ_ == 0) {
      if (pendingJog_) {
        requestedDirection_ = pendingDirection_;
        commandSpeed_ = pendingSpeed_;
        acceleration_ = pendingAcceleration_;
        pendingJog_ = false;
        mode_ = Mode::Continuous;
        phase_ = Phase::Stopped;
        return result;
      }
      immediateStop();
      return result;
    }
    if (directionSettleTicks_ > 0) {
      --directionSettleTicks_;
      return result;
    }

    phaseAccumulator_ += speedQ_;
    const std::uint64_t threshold =
        static_cast<std::uint64_t>(tickRateHz_) * kRateScale;
    if (phaseAccumulator_ >= threshold && speedQ_ > 0) {
      phaseAccumulator_ -= threshold;
      position_ += direction_;
      result.step = true;
      result.direction = direction_;
      if (mode_ == Mode::Finite && position_ == target_) immediateStop();
    }
    return result;
  }

  Snapshot snapshot() const {
    Snapshot out;
    out.mode = mode_;
    out.phase = phase_;
    out.position = position_;
    out.target = target_;
    out.speed = static_cast<std::uint32_t>((speedQ_ + kRateScale / 2) / kRateScale);
    out.commandSpeed = commandSpeed_;
    out.acceleration = acceleration_;
    out.direction = direction_;
    out.remaining = mode_ == Mode::Finite ? distance(position_, target_) : 0;
    out.moveTotal = mode_ == Mode::Finite ? distance(moveStart_, target_) : 0;
    return out;
  }

 private:
  std::uint32_t tickRateHz_;
  Mode mode_ = Mode::Idle;
  Phase phase_ = Phase::Stopped;
  std::int64_t position_ = 0;
  std::int64_t target_ = 0;
  std::int64_t moveStart_ = 0;
  std::uint64_t speedQ_ = 0;
  std::uint64_t phaseAccumulator_ = 0;
  std::uint32_t commandSpeed_ = 0;
  std::uint32_t acceleration_ = 1;
  std::int8_t direction_ = 1;
  std::int8_t requestedDirection_ = 1;
  std::uint8_t directionSettleTicks_ = 0;
  bool pendingJog_ = false;
  std::int8_t pendingDirection_ = 1;
  std::uint32_t pendingSpeed_ = 0;
  std::uint32_t pendingAcceleration_ = 1;

  bool validCommand(std::uint32_t speed, std::uint32_t acceleration) const {
    return speed > 0 && speed <= tickRateHz_ / 2 && acceleration > 0;
  }

  std::uint32_t accelerationIncrement(std::uint32_t acceleration) const {
    const std::uint64_t scaled = static_cast<std::uint64_t>(acceleration) * kRateScale;
    return static_cast<std::uint32_t>(std::max<std::uint64_t>(1, scaled / tickRateHz_));
  }

  static std::uint64_t subtractFloor(std::uint64_t value, std::uint64_t amount) {
    return value > amount ? value - amount : 0;
  }

  static std::uint64_t distance(std::int64_t from, std::int64_t to) {
    return to >= from ? static_cast<std::uint64_t>(to) - static_cast<std::uint64_t>(from)
                      : static_cast<std::uint64_t>(from) - static_cast<std::uint64_t>(to);
  }

  static std::uint64_t integerSqrt(std::uint64_t value) {
    std::uint64_t result = 0;
    std::uint64_t bit = 1ULL << 62;
    while (bit > value) bit >>= 2;
    while (bit != 0) {
      if (value >= result + bit) {
        value -= result + bit;
        result = (result >> 1) + bit;
      } else {
        result >>= 1;
      }
      bit >>= 2;
    }
    return result;
  }
};
