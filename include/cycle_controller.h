#pragma once

#include <cstdint>

template <typename Engine>
class CycleController {
 public:
  enum class Phase : uint8_t { Idle, MovingOut, DwellingOut, MovingBack, DwellingBack };

  bool start(Engine& engine, uint64_t travel, int8_t direction,
             uint32_t speed, uint32_t acceleration, uint32_t dwellMs,
             uint32_t repeats, bool infinite, uint32_t now) {
    if (travel == 0 || travel > static_cast<uint64_t>(INT64_MAX) || speed == 0 ||
        acceleration == 0 || (!infinite && repeats == 0)) {
      return false;
    }
    stop(engine, true);
    const auto state = engine.snapshot();
    const int64_t signedTravel =
        direction >= 0 ? static_cast<int64_t>(travel) : -static_cast<int64_t>(travel);
    if ((signedTravel > 0 && state.position > INT64_MAX - signedTravel) ||
        (signedTravel < 0 && state.position < INT64_MIN - signedTravel)) {
      return false;
    }
    active_ = true;
    infinite_ = infinite;
    phase_ = Phase::MovingOut;
    startPosition_ = state.position;
    outPosition_ = state.position + signedTravel;
    speed_ = speed;
    acceleration_ = acceleration;
    dwellMs_ = dwellMs;
    repeats_ = repeats;
    completed_ = 0;
    return engine.moveTo(outPosition_, speed_, acceleration_);
  }

  void stop(Engine& engine, bool immediate = false) {
    active_ = false;
    infinite_ = false;
    phase_ = Phase::Idle;
    if (immediate) engine.stop();
  }

  void update(Engine& engine, uint32_t now) {
    if (!active_) return;
    switch (phase_) {
      case Phase::MovingOut:
        if (engine.idle()) {
          phase_ = Phase::DwellingOut;
          deadline_ = now + dwellMs_;
        }
        break;
      case Phase::DwellingOut:
        if (due(now, deadline_)) {
          phase_ = Phase::MovingBack;
          engine.moveTo(startPosition_, speed_, acceleration_);
        }
        break;
      case Phase::MovingBack:
        if (engine.idle()) {
          ++completed_;
          if (!infinite_ && completed_ >= repeats_) {
            active_ = false;
            phase_ = Phase::Idle;
          } else {
            phase_ = Phase::DwellingBack;
            deadline_ = now + dwellMs_;
          }
        }
        break;
      case Phase::DwellingBack:
        if (due(now, deadline_)) {
          phase_ = Phase::MovingOut;
          engine.moveTo(outPosition_, speed_, acceleration_);
        }
        break;
      case Phase::Idle:
        break;
    }
  }

  bool active() const { return active_; }
  bool infinite() const { return infinite_; }
  Phase phase() const { return phase_; }
  uint32_t completed() const { return completed_; }

 private:
  bool active_ = false;
  bool infinite_ = false;
  Phase phase_ = Phase::Idle;
  int64_t startPosition_ = 0;
  int64_t outPosition_ = 0;
  uint32_t speed_ = 0;
  uint32_t acceleration_ = 0;
  uint32_t dwellMs_ = 0;
  uint32_t repeats_ = 0;
  uint32_t completed_ = 0;
  uint32_t deadline_ = 0;

  static bool due(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
  }
};
