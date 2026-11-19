#pragma once

#include <cstdint>

namespace AppConfig {

constexpr std::uint32_t kTelemetryPeriodMs = 100;
constexpr std::uint32_t kDiagnosticsPeriodMs = 500;
constexpr std::uint32_t kWifiRetryMs = 10000;
constexpr std::uint32_t kMaxAcceleration = 1000000;
constexpr std::uint32_t kMaxMoveSteps = 100000000;
constexpr std::uint32_t kMaxDwellMs = 3600000;

#ifndef MOTION_TIMER_HZ
#define MOTION_TIMER_HZ 50000
#endif
#ifndef MAX_STEP_RATE
#define MAX_STEP_RATE 10000
#endif
#ifndef STEP_PULSE_HIGH_US
#define STEP_PULSE_HIGH_US 200
#endif
#ifndef STEP_PULSE_LOW_US
#define STEP_PULSE_LOW_US 200
#endif
#ifndef DEFAULT_SPEED
#define DEFAULT_SPEED 400
#endif
#ifndef DEFAULT_ACCELERATION
#define DEFAULT_ACCELERATION 800
#endif
#ifndef DEFAULT_MAX_SPEED
#define DEFAULT_MAX_SPEED 1200
#endif
#ifndef DEFAULT_MAX_ACCELERATION
#define DEFAULT_MAX_ACCELERATION 10000
#endif

constexpr std::uint32_t kMotionTimerHz = MOTION_TIMER_HZ;
constexpr std::uint32_t kMaxStepRate = MAX_STEP_RATE;
constexpr std::uint32_t kStepPulseHighUs = STEP_PULSE_HIGH_US;
constexpr std::uint32_t kStepPulseLowUs = STEP_PULSE_LOW_US;
constexpr std::uint32_t kStepPulseHighTicks =
    (static_cast<std::uint64_t>(kMotionTimerHz) * kStepPulseHighUs + 999999) /
    1000000;
constexpr std::uint32_t kStepPulseLowTicks =
    (static_cast<std::uint64_t>(kMotionTimerHz) * kStepPulseLowUs + 999999) /
    1000000;
constexpr std::uint32_t kDefaultSpeed = DEFAULT_SPEED;
constexpr std::uint32_t kDefaultAcceleration = DEFAULT_ACCELERATION;
constexpr std::uint32_t kDefaultMaxSpeed = DEFAULT_MAX_SPEED;
constexpr std::uint32_t kDefaultMaxAcceleration = DEFAULT_MAX_ACCELERATION;

static_assert(kMaxStepRate <= kMotionTimerHz / 2,
              "MAX_STEP_RATE must leave at least one low timer tick per pulse");
static_assert(kStepPulseHighTicks > 0 && kStepPulseLowTicks > 0,
              "STEP pulse high and low times must each span at least one timer tick");
static_assert(kMaxStepRate <=
                  kMotionTimerHz / (kStepPulseHighTicks + kStepPulseLowTicks),
              "MAX_STEP_RATE is too high for the configured STEP pulse timing");
static_assert(kDefaultSpeed <= kDefaultMaxSpeed,
              "DEFAULT_SPEED exceeds DEFAULT_MAX_SPEED");

}  // namespace AppConfig
