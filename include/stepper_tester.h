#pragma once

#include <Arduino.h>

class StepperTester {
    public:
        StepperTester(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin, uint8_t ledPin);

        void begin();
        void runTestCycle();

    private:
        static constexpr uint16_t kStepsPerMove = 16000;
        static constexpr uint32_t kMoveIntervalMs = 1000;
        static constexpr uint32_t kDisabledIntervalMs = 5000;
        static constexpr uint32_t kPulseWidthUs = 200;
        static constexpr uint32_t kDirectionSetupUs = 10;

        void enableDriver();
        void disableDriver();
        void move(bool moveRight, uint16_t steps);
        void pulseStep();

        const uint8_t stepPin_;
        const uint8_t directionPin_;
        const uint8_t enablePin_;
        const uint8_t ledPin_;
};
