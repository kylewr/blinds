#include <unity.h>

#include "cycle_controller.h"
#include "driver_settings.h"
#include "failure_latch.h"
#include "motion_planner.h"
#include "servo_sweep.h"

#include <cstdint>

namespace {

constexpr std::uint32_t kTickRate = 50000;

void runUntilIdle(MotionPlanner& planner, std::uint32_t limit = 2000000) {
  for (std::uint32_t tick = 0; tick < limit; ++tick) {
    planner.tick();
    if (planner.snapshot().mode == MotionPlanner::Mode::Idle) return;
  }
  TEST_FAIL_MESSAGE("motion did not stop before the test limit");
}

void test_finite_motion_and_overflow_validation() {
  MotionPlanner planner(kTickRate);
  TEST_ASSERT_TRUE(planner.moveBy(137, 1200, 4000));
  runUntilIdle(planner);
  TEST_ASSERT_EQUAL_INT64(137, planner.snapshot().position);
  TEST_ASSERT_TRUE(planner.moveBy(-289, 1600, 5000));
  runUntilIdle(planner);
  TEST_ASSERT_EQUAL_INT64(-152, planner.snapshot().position);
  TEST_ASSERT_FALSE(planner.moveBy(1, 0, 100));
  TEST_ASSERT_FALSE(planner.moveBy(1, kTickRate / 2 + 1, 100));
  TEST_ASSERT_FALSE(planner.moveBy(1, 100, 0));
}

void test_jog_reversal_deceleration_and_emergency_stop() {
  MotionPlanner planner(kTickRate);
  TEST_ASSERT_TRUE(planner.jog(1, 1000, 5000));
  for (std::uint32_t tick = 0; tick < kTickRate / 2; ++tick) planner.tick();
  const auto forward = planner.snapshot();
  TEST_ASSERT_GREATER_THAN_INT64(0, forward.position);
  TEST_ASSERT_TRUE(planner.jog(-1, 1000, 5000));
  bool sawDeceleration = false;
  for (std::uint32_t tick = 0; tick < kTickRate; ++tick) {
    planner.tick();
    sawDeceleration |= planner.snapshot().phase == MotionPlanner::Phase::Decelerating;
  }
  TEST_ASSERT_TRUE(sawDeceleration);
  TEST_ASSERT_LESS_THAN_INT64(forward.position, planner.snapshot().position);
  planner.immediateStop();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(MotionPlanner::Mode::Idle),
                        static_cast<int>(planner.snapshot().mode));
  TEST_ASSERT_EQUAL_UINT32(0, planner.snapshot().speed);
}

void test_maximum_rate_never_exceeds_request() {
  MotionPlanner planner(kTickRate);
  TEST_ASSERT_TRUE(planner.jog(1, 10000, 1000000));
  std::uint32_t pulses = 0;
  for (std::uint32_t tick = 0; tick < kTickRate; ++tick) {
    if (planner.tick().step) ++pulses;
  }
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(10000, pulses);
  TEST_ASSERT_GREATER_THAN_UINT32(9000, pulses);
}

struct FakeState { std::int64_t position = 0; };

class FakeEngine {
 public:
  FakeState snapshot() const { return {position}; }
  bool moveTo(std::int64_t next, std::uint32_t, std::uint32_t) {
    target = next;
    idleValue = false;
    return true;
  }
  void stop() { idleValue = true; }
  bool idle() const { return idleValue; }
  void arrive() {
    position = target;
    idleValue = true;
  }
  std::int64_t position = 0;
  std::int64_t target = 0;
  bool idleValue = true;
};

void test_cycle_sequences_out_and_back() {
  FakeEngine engine;
  CycleController<FakeEngine> cycle;
  TEST_ASSERT_TRUE(cycle.start(engine, 100, 1, 400, 800, 20, 2, false, 0));
  TEST_ASSERT_EQUAL_INT64(100, engine.target);
  engine.arrive();
  cycle.update(engine, 10);
  cycle.update(engine, 29);
  TEST_ASSERT_EQUAL_INT64(100, engine.position);
  cycle.update(engine, 30);
  TEST_ASSERT_EQUAL_INT64(0, engine.target);
  engine.arrive();
  cycle.update(engine, 40);
  TEST_ASSERT_EQUAL_UINT32(1, cycle.completed());
  cycle.update(engine, 60);
  TEST_ASSERT_EQUAL_INT64(100, engine.target);
  engine.arrive();
  cycle.update(engine, 70);
  cycle.update(engine, 90);
  engine.arrive();
  cycle.update(engine, 100);
  TEST_ASSERT_FALSE(cycle.active());
  TEST_ASSERT_EQUAL_UINT32(2, cycle.completed());
}

void test_driver_settings_validation() {
  DriverSettings settings;
  TEST_ASSERT_EQUAL_UINT16(600, settings.runCurrentMa);
  TEST_ASSERT_EQUAL_UINT8(50, settings.holdPercent);
  TEST_ASSERT_EQUAL_UINT8(8, settings.holdDelay);
  TEST_ASSERT_EQUAL_UINT8(20, settings.powerDownDelay);
  TEST_ASSERT_EQUAL_UINT16(16, settings.microsteps);
  TEST_ASSERT_TRUE(settings.interpolation);
  TEST_ASSERT_FALSE(settings.doubleEdge);
  TEST_ASSERT_TRUE(settings.multistepFilter);
  TEST_ASSERT_FALSE(settings.spreadCycle);
  TEST_ASSERT_FALSE(settings.shaftInverted);
  TEST_ASSERT_EQUAL_UINT32(0, settings.stealthThreshold);
  TEST_ASSERT_EQUAL_UINT8(4, settings.offTime);
  TEST_ASSERT_EQUAL_UINT8(24, settings.blankTime);
  TEST_ASSERT_EQUAL_INT8(0, settings.hysteresisEnd);
  TEST_ASSERT_EQUAL_UINT8(1, settings.hysteresisStart);
  TEST_ASSERT_EQUAL_UINT8(36, settings.pwmOffset);
  TEST_ASSERT_EQUAL_UINT8(14, settings.pwmGradient);
  TEST_ASSERT_EQUAL_UINT8(1, settings.pwmFrequency);
  TEST_ASSERT_TRUE(settings.pwmAutoscale);
  TEST_ASSERT_TRUE(settings.pwmAutograd);
  TEST_ASSERT_EQUAL_UINT8(0, settings.freewheel);
  TEST_ASSERT_EQUAL_UINT8(8, settings.pwmRegulation);
  TEST_ASSERT_EQUAL_UINT8(12, settings.pwmLimit);
  TEST_ASSERT_EQUAL_UINT32(0, settings.coolThreshold);
  TEST_ASSERT_EQUAL_UINT8(0, settings.stallThreshold);
  TEST_ASSERT_EQUAL_UINT8(0, settings.coolMinimum);
  TEST_ASSERT_EQUAL_UINT8(0, settings.coolUpStep);
  TEST_ASSERT_EQUAL_UINT8(0, settings.coolMaximum);
  TEST_ASSERT_EQUAL_UINT8(0, settings.coolDownStep);
  TEST_ASSERT_FALSE(settings.coolMinimumQuarter);
  TEST_ASSERT_EQUAL_UINT8(2, settings.uartReplyDelay);
  TEST_ASSERT_TRUE(DriverSettings::validate(settings));
  settings.microsteps = 3;
  TEST_ASSERT_FALSE(DriverSettings::validate(settings));
  settings = DriverSettings{};
  settings.runCurrentMa = 2001;
  TEST_ASSERT_FALSE(DriverSettings::validate(settings));
  settings = DriverSettings{};
  DriverSettings changed = settings;
  changed.microsteps = 32;
  TEST_ASSERT_TRUE(changed.changesStepInterpretation(settings));
}

void test_uart_failure_latches_after_three_consecutive_failures() {
  ConsecutiveFailureLatch latch(3);
  latch.failure();
  latch.failure();
  TEST_ASSERT_FALSE(latch.latched());
  latch.success();
  TEST_ASSERT_EQUAL_UINT8(0, latch.count());
  latch.failure();
  latch.failure();
  latch.failure();
  TEST_ASSERT_TRUE(latch.latched());
  latch.success();
  TEST_ASSERT_TRUE(latch.latched());
  latch.clear();
  TEST_ASSERT_FALSE(latch.latched());
}

void test_servo_mapping_clamping_and_sweep() {
  TEST_ASSERT_EQUAL_UINT16(500, ServoSweep::angleToPulse(0, 500, 2500));
  TEST_ASSERT_EQUAL_UINT16(1500, ServoSweep::angleToPulse(135, 500, 2500));
  TEST_ASSERT_EQUAL_UINT16(2500, ServoSweep::angleToPulse(270, 500, 2500));
  TEST_ASSERT_EQUAL_UINT16(500, ServoSweep::clampPulse(400, 500, 2500));
  TEST_ASSERT_EQUAL_UINT16(2500, ServoSweep::clampPulse(2600, 500, 2500));

  ServoSweep sweep;
  ServoSweep::Settings settings;
  settings.travelMs = 1000;
  settings.dwellMs = 100;
  settings.repeats = 1;
  TEST_ASSERT_TRUE(sweep.start(settings, 0));
  TEST_ASSERT_EQUAL_UINT16(1500, sweep.update(500));
  TEST_ASSERT_EQUAL_UINT16(2500, sweep.update(1000));
  sweep.update(1100);
  TEST_ASSERT_EQUAL_UINT16(1500, sweep.update(1600));
  TEST_ASSERT_EQUAL_UINT16(500, sweep.update(2100));
  TEST_ASSERT_FALSE(sweep.active());
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_finite_motion_and_overflow_validation);
  RUN_TEST(test_jog_reversal_deceleration_and_emergency_stop);
  RUN_TEST(test_maximum_rate_never_exceeds_request);
  RUN_TEST(test_cycle_sequences_out_and_back);
  RUN_TEST(test_driver_settings_validation);
  RUN_TEST(test_uart_failure_latches_after_three_consecutive_failures);
  RUN_TEST(test_servo_mapping_clamping_and_sweep);
  return UNITY_END();
}
