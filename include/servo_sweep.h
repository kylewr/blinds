#pragma once

#include <algorithm>
#include <cstdint>

class ServoSweep {
 public:
  static constexpr std::uint16_t kMaximumAngle = 270;

  struct Settings {
    std::uint16_t minimumUs = 500;
    std::uint16_t maximumUs = 2500;
    std::uint16_t startUs = 500;
    std::uint16_t endUs = 2500;
    std::uint32_t travelMs = 1000;
    std::uint32_t dwellMs = 250;
    std::uint32_t repeats = 1;
    bool infinite = false;
  };

  enum class Phase : std::uint8_t { Idle, Out, DwellOut, Back, DwellBack };

  static bool validate(const Settings& value) {
    return value.minimumUs >= 500 && value.maximumUs <= 2500 &&
           value.minimumUs < value.maximumUs && value.startUs >= value.minimumUs &&
           value.endUs <= value.maximumUs && value.startUs < value.endUs &&
           value.travelMs > 0 && value.travelMs <= 3600000 &&
           value.dwellMs <= 3600000 && (value.infinite || value.repeats > 0);
  }

  static std::uint16_t angleToPulse(std::uint16_t angle, std::uint16_t minimumUs,
                                    std::uint16_t maximumUs) {
    const std::uint16_t clamped = std::min(kMaximumAngle, angle);
    return minimumUs + static_cast<std::uint32_t>(maximumUs - minimumUs) * clamped /
                           kMaximumAngle;
  }

  static std::uint16_t clampPulse(std::uint16_t pulse, std::uint16_t minimumUs,
                                  std::uint16_t maximumUs) {
    return std::max(minimumUs, std::min(maximumUs, pulse));
  }

  bool start(const Settings& value, std::uint32_t now) {
    if (!validate(value)) return false;
    settings_ = value;
    phase_ = Phase::Out;
    phaseStartedAt_ = now;
    completed_ = 0;
    active_ = true;
    currentUs_ = settings_.startUs;
    return true;
  }

  void stop() {
    active_ = false;
    phase_ = Phase::Idle;
  }

  std::uint16_t update(std::uint32_t now) {
    if (!active_) return currentUs_;
    const std::uint32_t elapsed = now - phaseStartedAt_;
    switch (phase_) {
      case Phase::Out:
        currentUs_ = interpolate(settings_.startUs, settings_.endUs, elapsed);
        if (elapsed >= settings_.travelMs) transition(Phase::DwellOut, now);
        break;
      case Phase::DwellOut:
        currentUs_ = settings_.endUs;
        if (elapsed >= settings_.dwellMs) transition(Phase::Back, now);
        break;
      case Phase::Back:
        currentUs_ = interpolate(settings_.endUs, settings_.startUs, elapsed);
        if (elapsed >= settings_.travelMs) {
          ++completed_;
          if (!settings_.infinite && completed_ >= settings_.repeats) {
            stop();
          } else {
            transition(Phase::DwellBack, now);
          }
        }
        break;
      case Phase::DwellBack:
        currentUs_ = settings_.startUs;
        if (elapsed >= settings_.dwellMs) transition(Phase::Out, now);
        break;
      case Phase::Idle:
        break;
    }
    return currentUs_;
  }

  bool active() const { return active_; }
  bool infinite() const { return settings_.infinite; }
  Phase phase() const { return phase_; }
  std::uint16_t currentUs() const { return currentUs_; }
  std::uint32_t completed() const { return completed_; }

 private:
  Settings settings_{};
  Phase phase_ = Phase::Idle;
  std::uint32_t phaseStartedAt_ = 0;
  std::uint32_t completed_ = 0;
  std::uint16_t currentUs_ = 1500;
  bool active_ = false;

  std::uint16_t interpolate(std::uint16_t from, std::uint16_t to,
                            std::uint32_t elapsed) const {
    if (elapsed >= settings_.travelMs) return to;
    const std::int32_t distance = static_cast<std::int32_t>(to) - from;
    return static_cast<std::uint16_t>(
        static_cast<std::int32_t>(from) +
        distance * static_cast<std::int64_t>(elapsed) / settings_.travelMs);
  }

  void transition(Phase phase, std::uint32_t now) {
    phase_ = phase;
    phaseStartedAt_ = now;
  }
};
