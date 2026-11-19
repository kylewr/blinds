#include "tmc2209_uart.h"

Tmc2209Uart::Tmc2209Uart(HardwareSerial& serial, uint8_t receivePin,
                         uint8_t transmitPin, uint16_t rmsCurrentMilliamps,
                         uint16_t microsteps)
    : serial_(serial),
      driver_(&serial_, kSenseResistorOhms, kDriverAddress),
      receivePin_(receivePin),
      transmitPin_(transmitPin) {
  settings_.runCurrentMa = rmsCurrentMilliamps;
  settings_.microsteps = microsteps;
}

void Tmc2209Uart::begin() {
  serial_.begin(kBaudRate, SERIAL_8N1, receivePin_, transmitPin_);
  driver_.begin();
  retry();
}

bool Tmc2209Uart::retry() {
  diagnostics_.uartRxIdleHigh = digitalRead(receivePin_) == HIGH;
  diagnostics_.uartEcho = probeUartEcho();
  driver_.CRCerror = false;
  const uint8_t code = driver_.test_connection();
  diagnostics_.connectionCode = code;
  if (code != 0 || driver_.CRCerror) {
    diagnostics_.detectedAddress = scanAddresses(diagnostics_.detectedBaud);
    serial_.updateBaudRate(kBaudRate);
    recordFailure(code == 0 ? 2 : code);
    return false;
  }
  diagnostics_.detectedAddress = kDriverAddress;
  diagnostics_.detectedBaud = kBaudRate;
  diagnostics_.connected = true;
  failures_.clear();
  diagnostics_.faultLatched = failures_.latched();
  diagnostics_.consecutiveFailures = failures_.count();
  return applySettings(settings_) && pollDiagnostics();
}

bool Tmc2209Uart::applySettings(const DriverSettings& settings) {
  if (!DriverSettings::validate(settings) || !diagnostics_.connected) return false;
  driver_.CRCerror = false;
  driver_.pdn_disable(true);
  driver_.mstep_reg_select(true);
  driver_.I_scale_analog(false);
  driver_.internal_Rsense(false);
  driver_.rms_current(settings.runCurrentMa,
                      static_cast<float>(settings.holdPercent) / 100.0F);
  driver_.iholddelay(settings.holdDelay);
  driver_.TPOWERDOWN(settings.powerDownDelay);
  driver_.microsteps(settings.microsteps == 1 ? 0 : settings.microsteps);
  driver_.intpol(settings.interpolation);
  driver_.dedge(settings.doubleEdge);
  driver_.multistep_filt(settings.multistepFilter);
  driver_.en_spreadCycle(settings.spreadCycle);
  driver_.shaft(settings.shaftInverted);
  driver_.TPWMTHRS(settings.stealthThreshold);
  driver_.toff(settings.offTime);
  driver_.blank_time(settings.blankTime);
  driver_.hysteresis_end(settings.hysteresisEnd);
  driver_.hysteresis_start(settings.hysteresisStart);
  driver_.pwm_ofs(settings.pwmOffset);
  driver_.pwm_grad(settings.pwmGradient);
  driver_.pwm_freq(settings.pwmFrequency);
  driver_.pwm_autoscale(settings.pwmAutoscale);
  driver_.pwm_autograd(settings.pwmAutograd);
  driver_.freewheel(settings.freewheel);
  driver_.pwm_reg(settings.pwmRegulation);
  driver_.pwm_lim(settings.pwmLimit);
  driver_.TCOOLTHRS(settings.coolThreshold);
  driver_.SGTHRS(settings.stallThreshold);
  driver_.semin(settings.coolMinimum);
  driver_.seup(settings.coolUpStep);
  driver_.semax(settings.coolMaximum);
  driver_.sedn(settings.coolDownStep);
  driver_.seimin(settings.coolMinimumQuarter);
  driver_.senddelay(settings.uartReplyDelay);
  if (driver_.CRCerror) {
    recordFailure(2);
    return false;
  }
  settings_ = settings;
  return true;
}

bool Tmc2209Uart::pollDiagnostics() {
  driver_.CRCerror = false;
  const uint8_t code = driver_.test_connection();
  diagnostics_.connectionCode = code;
  if (code != 0 || driver_.CRCerror) {
    recordFailure(code == 0 ? 2 : code);
    return false;
  }
  readDiagnosticRegisters();
  if (driver_.CRCerror) {
    recordFailure(2);
    return false;
  }
  diagnostics_.connected = true;
  failures_.success();
  diagnostics_.consecutiveFailures = failures_.count();
  diagnostics_.faultLatched = failures_.latched();
  return true;
}

void Tmc2209Uart::clearStatus() {
  if (!diagnostics_.connected) return;
  driver_.GSTAT(0x07);
  pollDiagnostics();
}

const DriverSettings& Tmc2209Uart::settings() const { return settings_; }
const TmcDiagnostics& Tmc2209Uart::diagnostics() const { return diagnostics_; }
bool Tmc2209Uart::healthy() const {
  return diagnostics_.connected && !diagnostics_.faultLatched;
}
bool Tmc2209Uart::faultLatched() const { return diagnostics_.faultLatched; }

bool Tmc2209Uart::probeUartEcho() {
  while (serial_.available() > 0) serial_.read();
  constexpr uint8_t kProbeByte = 0xFF;
  serial_.write(kProbeByte);
  serial_.flush();
  delay(2);
  bool echoed = false;
  while (serial_.available() > 0) {
    if (serial_.read() == kProbeByte) echoed = true;
  }
  delay(2);
  return echoed;
}

int8_t Tmc2209Uart::scanAddresses(uint32_t& detectedBaud) {
  constexpr uint32_t kBaudRates[] = {115200, 57600, 38400, 19200, 9600};
  detectedBaud = 0;
  for (const uint32_t baud : kBaudRates) {
    serial_.updateBaudRate(baud);
    delay(2);
    for (uint8_t address = 0; address < 4; ++address) {
      TMC2209Stepper probe(&serial_, kSenseResistorOhms, address);
      probe.CRCerror = false;
      if (probe.test_connection() == 0 && !probe.CRCerror) {
        detectedBaud = baud;
        return static_cast<int8_t>(address);
      }
    }
  }
  return -1;
}

void Tmc2209Uart::recordFailure(uint8_t connectionCode) {
  diagnostics_.connectionCode = connectionCode;
  diagnostics_.connected = false;
  failures_.failure();
  diagnostics_.consecutiveFailures = failures_.count();
  diagnostics_.faultLatched = failures_.latched();
}

void Tmc2209Uart::readDiagnosticRegisters() {
  diagnostics_.gconf = driver_.GCONF();
  diagnostics_.globalStatus = driver_.GSTAT();
  diagnostics_.interfaceCount = driver_.IFCNT();
  diagnostics_.inputStatus = driver_.IOIN();
  diagnostics_.version = static_cast<uint8_t>(diagnostics_.inputStatus >> 24);
  diagnostics_.slaveConfig = driver_.SLAVECONF();
  diagnostics_.otpRead = driver_.OTP_READ();
  diagnostics_.factoryConfig = driver_.FACTORY_CONF();
  diagnostics_.currentConfig = driver_.IHOLD_IRUN();
  diagnostics_.powerDown = driver_.TPOWERDOWN();
  diagnostics_.actualVelocity = driver_.VACTUAL();
  diagnostics_.tstep = driver_.TSTEP();
  diagnostics_.stealthThreshold = driver_.TPWMTHRS();
  diagnostics_.coolThreshold = driver_.TCOOLTHRS();
  diagnostics_.stallThreshold = driver_.SGTHRS();
  diagnostics_.stallResult = driver_.SG_RESULT();
  diagnostics_.coolConfig = driver_.COOLCONF();
  diagnostics_.microstepCounter = driver_.MSCNT();
  diagnostics_.microstepCurrents = driver_.MSCURACT();
  diagnostics_.coilCurrentA = driver_.cur_a();
  diagnostics_.coilCurrentB = driver_.cur_b();
  diagnostics_.chopperConfig = driver_.CHOPCONF();
  diagnostics_.driverStatus = driver_.DRV_STATUS();
  diagnostics_.pwmConfig = driver_.PWMCONF();
  diagnostics_.pwmScale = driver_.PWM_SCALE();
  diagnostics_.pwmScaleSum = diagnostics_.pwmScale & 0xFF;
  diagnostics_.pwmScaleAuto = driver_.pwm_scale_auto();
  diagnostics_.pwmAuto = driver_.PWM_AUTO();
  diagnostics_.pwmOffsetAuto = diagnostics_.pwmAuto & 0xFF;
  diagnostics_.pwmGradientAuto = (diagnostics_.pwmAuto >> 16) & 0xFF;
}
