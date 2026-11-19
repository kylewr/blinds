#include <Arduino.h>

#include "stepper_tester.h"
#include "tmc2209_uart.h"

#if !defined(PIN_STEP) || !defined(PIN_DIR) || !defined(PIN_ENABLE) || \
    !defined(PIN_LED) || !defined(PIN_UART_TX) || !defined(PIN_UART_RX) || \
    !defined(TMC_RMS_CURRENT_MA) || !defined(TMC_MICROSTEPS)
#error "Stepper pins and TMC UART settings must be set in platformio.ini"
#endif

namespace {
HardwareSerial tmcSerial(1);
Tmc2209Uart tmcDriver(tmcSerial, PIN_UART_RX, PIN_UART_TX,
                      TMC_RMS_CURRENT_MA, TMC_MICROSTEPS);
StepperTester stepper(PIN_STEP, PIN_DIR, PIN_ENABLE, PIN_LED);
}  // namespace

void setup() {
  tmcDriver.begin();
  stepper.begin();
}

void loop() {
  stepper.runTestCycle();
}
