# ESP32-C3 blinds test bench

This project contains two independent ESP32 firmware targets for a TMC2209
stepper driver:

- `esp32-c3-devkitc-02` is the original automatic left/right tester.
- `esp32-c3-web` is the LAN web test bench with accelerated motion, complete
  safe TMC2209 runtime configuration and diagnostics, and a PWM servo output.

## Wiring

| Function | ESP32-C3 GPIO | Notes |
| --- | ---: | --- |
| TMC DIR | 0 | Direct |
| TMC STEP | 1 | Direct |
| TMC ENABLE | 3 | Active-low |
| TMC UART TX | 4 | Through 1 kΩ to PDN_UART |
| TMC UART RX | 5 | Directly to the same PDN_UART node |
| Status LED | 8 | Active-low; follows driver enable |
| Servo signal | 10 | 100 Hz PWM signal only |

The TMC2209 configuration assumes UART address 0 and a 0.11 Ω sense resistor.
Power the servo from a suitable external 5 V supply and connect its ground to
the ESP32 ground. Do not power the servo from the ESP32 3.3 V pin.

## Configure Wi-Fi

Edit `platformio.local.ini` and put the trusted-LAN SSID and password between
the escaped quotes. `platformio.local.ini.example` shows the expected format.
The local file is ignored so credentials are not committed.

The web target is station-only: it does not create an access point. After it
joins the LAN, open `http://blinds-test.local/`. The serial monitor at 115200
baud can be used while diagnosing network startup.

## Build and upload

```sh
# Original automatic tester
pio run -e esp32-c3-devkitc-02
pio run -e esp32-c3-devkitc-02 -t upload

# Web test bench
pio run -e esp32-c3-web
pio run -e esp32-c3-web -t upload

# Hardware-independent tests
pio test -e native
```

## Web test bench behavior

The motor driver and servo signal are off after every boot. Motion controls
include held or latched jog, relative and absolute moves, software zero,
direction inversion, configurable speed/acceleration limits, and repeating
forward/back cycles. Position is calculated from emitted pulses and is not a
closed-loop measurement.

The TMC panel exposes current, microstepping, StealthChop/SpreadCycle, chopper
timing, PWM tuning, StallGuard, CoolStep, UART timing, and live status/register
diagnostics. Applying settings stops and disables the motor. OTP programming,
factory-trim writes, test mode, protection-disable bits, direct `VACTUAL`
motion, and arbitrary writes are intentionally unavailable.

The servo panel supports attach/detach, 0–270° angle and pulse-width commands,
500–2500 µs default endpoints, and finite or continuous 100 Hz sweeps. Pressing Space
or **STOP ALL** immediately disables the stepper and detaches the servo.

All UI settings are session-only. Rebooting restores compile-time defaults and
does not restore either output.

The web motion defaults use 16,000-step travel, 20 µs STEP high plus at least
20 µs low, 10,000 steps/s, 100,000 steps/s² acceleration, and a 1-second
direction dwell. The 50 kHz motion timer permits up to 25,000 steps/s.

## Safety

- Start servo endpoint testing with the mechanical linkage disconnected.
- Set the TMC RMS current for the connected motor before extended operation.
- StallGuard is diagnostic assistance, not a substitute for limit switches.
- The HTTP/WebSocket interface is unauthenticated; keep it on a trusted LAN.
- A physical motor-power cutoff is still required for an emergency stop.
