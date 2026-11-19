#include "stepper_tester.h"

namespace {
    constexpr uint8_t kDriverEnabled = LOW;
    constexpr uint8_t kDriverDisabled = HIGH;
    constexpr uint8_t kLedOn = LOW;
    constexpr uint8_t kLedOff = HIGH;
    constexpr uint8_t kMoveLeft = LOW;
    constexpr uint8_t kMoveRight = HIGH;
} // namespace

StepperTester::StepperTester(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin,
                             uint8_t ledPin)
    : stepPin_(stepPin), directionPin_(directionPin), enablePin_(enablePin), ledPin_(ledPin) {}

void StepperTester::begin() {
    // Establish safe output levels before enabling any driver output.
    digitalWrite(stepPin_, LOW);
    digitalWrite(directionPin_, kMoveLeft);
    digitalWrite(enablePin_, kDriverDisabled);
    digitalWrite(ledPin_, kLedOff);

    pinMode(stepPin_, OUTPUT);
    pinMode(directionPin_, OUTPUT);
    pinMode(enablePin_, OUTPUT);
    pinMode(ledPin_, OUTPUT);

    enableDriver();
}

void StepperTester::runTestCycle() {
    delay(kMoveIntervalMs);
    move(false, kStepsPerMove);

    delay(kMoveIntervalMs);
    move(true, kStepsPerMove);

    disableDriver();
    delay(kDisabledIntervalMs);
    enableDriver();
}

void StepperTester::enableDriver() {
    digitalWrite(enablePin_, kDriverEnabled);
    digitalWrite(ledPin_, kLedOn);
}

void StepperTester::disableDriver() {
    digitalWrite(enablePin_, kDriverDisabled);
    digitalWrite(ledPin_, kLedOff);
}

void StepperTester::move(bool moveRight, uint16_t steps) {
    digitalWrite(directionPin_, moveRight ? kMoveRight : kMoveLeft);
    delayMicroseconds(kDirectionSetupUs);

    for (uint16_t step = 0; step < steps; ++step) {
        pulseStep();
    }
}

void StepperTester::pulseStep() {
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(kPulseWidthUs);
    digitalWrite(stepPin_, LOW);
    delayMicroseconds(kPulseWidthUs);
}
