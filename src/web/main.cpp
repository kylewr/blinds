#include "app_config.h"
#include "cycle_controller.h"
#include "motion_engine.h"
#include "servo_controller.h"
#include "tmc2209_uart.h"
#include "web_ui.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <WiFi.h>

#include <cstring>

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

#if !defined(PIN_STEP) || !defined(PIN_DIR) || !defined(PIN_ENABLE) || \
    !defined(PIN_LED) || !defined(PIN_UART_TX) || !defined(PIN_UART_RX) || \
    !defined(PIN_SERVO) || !defined(TMC_RMS_CURRENT_MA) || \
    !defined(TMC_MICROSTEPS)
#error "Web test bench pin and TMC build flags are incomplete"
#endif

namespace {

constexpr size_t kCommandBytes = 3072;
constexpr size_t kCommandQueueLength = 8;

enum class NetworkEventType : uint8_t { Connected, Disconnected, Message };

struct NetworkEvent {
  NetworkEventType type = NetworkEventType::Message;
  uint32_t clientId = 0;
  char json[kCommandBytes]{};
};

HardwareSerial tmcSerial(1);
Tmc2209Uart tmc(tmcSerial, PIN_UART_RX, PIN_UART_TX, TMC_RMS_CURRENT_MA,
                TMC_MICROSTEPS);
MotionEngine motion(PIN_STEP, PIN_DIR, PIN_ENABLE, PIN_LED,
                    AppConfig::kMotionTimerHz, AppConfig::kMaxStepRate,
                    AppConfig::kStepPulseHighTicks);
CycleController<MotionEngine> cycle;
ServoController servo(PIN_SERVO);
AsyncWebServer server(80);
AsyncWebSocket websocket("/ws");
QueueHandle_t commandQueue = nullptr;

uint32_t jogOwner = 0;
uint32_t sweepOwner = 0;
uint32_t lastTelemetryAt = 0;
uint32_t lastDiagnosticsAt = 0;
uint32_t lastWifiAttemptAt = 0;
bool mdnsStarted = false;
bool handledTmcFault = false;
uint32_t motionMaxSpeed = AppConfig::kDefaultMaxSpeed;
uint32_t motionMaxAcceleration = AppConfig::kDefaultMaxAcceleration;

const char* modeName(MotionPlanner::Mode mode) {
  switch (mode) {
    case MotionPlanner::Mode::Idle: return "stopped";
    case MotionPlanner::Mode::Finite: return "finite move";
    case MotionPlanner::Mode::Continuous: return "jogging";
    case MotionPlanner::Mode::Stopping: return "decelerating";
  }
  return "unknown";
}

void sendDocument(uint32_t clientId, JsonDocument& document) {
  AsyncWebSocketClient* client = websocket.client(clientId);
  if (client == nullptr || !client->canSend()) return;
  String output;
  serializeJson(document, output);
  client->text(output);
}

void broadcastDocument(JsonDocument& document) {
  String output;
  serializeJson(document, output);
  websocket.textAll(output);
}

void sendError(uint32_t clientId, const String& requestId, const char* code,
               const char* message) {
  JsonDocument document;
  document["v"] = 1;
  document["type"] = "error";
  document["id"] = requestId;
  document["code"] = code;
  document["message"] = message;
  sendDocument(clientId, document);
}

void sendAck(uint32_t clientId, const String& requestId) {
  JsonDocument document;
  document["v"] = 1;
  document["type"] = "ack";
  document["id"] = requestId;
  sendDocument(clientId, document);
}

void broadcastEvent(const char* message) {
  JsonDocument document;
  document["v"] = 1;
  document["type"] = "event";
  document["message"] = message;
  broadcastDocument(document);
}

void addMotionState(JsonDocument& document) {
  const auto state = motion.snapshot();
  JsonObject object = document["motion"].to<JsonObject>();
  object["mode"] = modeName(state.mode);
  object["phase"] = static_cast<uint8_t>(state.phase);
  object["position"] = state.position;
  object["target"] = state.target;
  object["speed"] = state.speed;
  object["commandSpeed"] = state.commandSpeed;
  object["acceleration"] = state.acceleration;
  object["remaining"] = state.remaining;
  object["moveTotal"] = state.moveTotal;
  object["direction"] = state.direction;
  object["enabled"] = motion.enabled();
  object["referenced"] = motion.referenced();
  object["directionInverted"] = motion.directionInverted();

  JsonObject cycleJson = document["cycle"].to<JsonObject>();
  cycleJson["active"] = cycle.active();
  cycleJson["infinite"] = cycle.infinite();
  cycleJson["phase"] = static_cast<uint8_t>(cycle.phase());
  cycleJson["completed"] = cycle.completed();
}

void addServoState(JsonDocument& document) {
  JsonObject object = document["servo"].to<JsonObject>();
  object["attached"] = servo.attached();
  object["minimumUs"] = servo.minimumUs();
  object["maximumUs"] = servo.maximumUs();
  object["pulseUs"] = servo.pulseUs();
  object["angle"] = servo.angle();
  object["sweepActive"] = servo.sweep().active();
  object["sweepInfinite"] = servo.sweep().infinite();
  object["sweepPhase"] = static_cast<uint8_t>(servo.sweep().phase());
  object["sweepCompleted"] = servo.sweep().completed();
}

void addDriverSettings(JsonDocument& document) {
  const auto& value = tmc.settings();
  JsonObject object = document["driverSettings"].to<JsonObject>();
#define ADD_SETTING(name) object[#name] = value.name
  ADD_SETTING(runCurrentMa);
  ADD_SETTING(holdPercent);
  ADD_SETTING(holdDelay);
  ADD_SETTING(powerDownDelay);
  ADD_SETTING(microsteps);
  ADD_SETTING(interpolation);
  ADD_SETTING(doubleEdge);
  ADD_SETTING(multistepFilter);
  ADD_SETTING(spreadCycle);
  ADD_SETTING(shaftInverted);
  ADD_SETTING(stealthThreshold);
  ADD_SETTING(offTime);
  ADD_SETTING(blankTime);
  ADD_SETTING(hysteresisEnd);
  ADD_SETTING(hysteresisStart);
  ADD_SETTING(pwmOffset);
  ADD_SETTING(pwmGradient);
  ADD_SETTING(pwmFrequency);
  ADD_SETTING(pwmAutoscale);
  ADD_SETTING(pwmAutograd);
  ADD_SETTING(freewheel);
  ADD_SETTING(pwmRegulation);
  ADD_SETTING(pwmLimit);
  ADD_SETTING(coolThreshold);
  ADD_SETTING(stallThreshold);
  ADD_SETTING(coolMinimum);
  ADD_SETTING(coolUpStep);
  ADD_SETTING(coolMaximum);
  ADD_SETTING(coolDownStep);
  ADD_SETTING(coolMinimumQuarter);
  ADD_SETTING(uartReplyDelay);
#undef ADD_SETTING
}

void addDiagnostics(JsonDocument& document) {
  const auto& value = tmc.diagnostics();
  JsonObject object = document["diagnostics"].to<JsonObject>();
#define ADD_DIAGNOSTIC(name) object[#name] = value.name
  ADD_DIAGNOSTIC(connected);
  ADD_DIAGNOSTIC(faultLatched);
  ADD_DIAGNOSTIC(uartEcho);
  ADD_DIAGNOSTIC(uartRxIdleHigh);
  ADD_DIAGNOSTIC(detectedAddress);
  ADD_DIAGNOSTIC(detectedBaud);
  ADD_DIAGNOSTIC(consecutiveFailures);
  ADD_DIAGNOSTIC(connectionCode);
  ADD_DIAGNOSTIC(interfaceCount);
  ADD_DIAGNOSTIC(version);
  ADD_DIAGNOSTIC(inputStatus);
  ADD_DIAGNOSTIC(globalStatus);
  ADD_DIAGNOSTIC(driverStatus);
  ADD_DIAGNOSTIC(tstep);
  ADD_DIAGNOSTIC(microstepCounter);
  ADD_DIAGNOSTIC(coilCurrentA);
  ADD_DIAGNOSTIC(coilCurrentB);
  ADD_DIAGNOSTIC(stallResult);
  ADD_DIAGNOSTIC(pwmScaleSum);
  ADD_DIAGNOSTIC(pwmScaleAuto);
  ADD_DIAGNOSTIC(pwmOffsetAuto);
  ADD_DIAGNOSTIC(pwmGradientAuto);
  ADD_DIAGNOSTIC(gconf);
  ADD_DIAGNOSTIC(slaveConfig);
  ADD_DIAGNOSTIC(otpRead);
  ADD_DIAGNOSTIC(factoryConfig);
  ADD_DIAGNOSTIC(currentConfig);
  ADD_DIAGNOSTIC(powerDown);
  ADD_DIAGNOSTIC(actualVelocity);
  ADD_DIAGNOSTIC(stealthThreshold);
  ADD_DIAGNOSTIC(coolThreshold);
  ADD_DIAGNOSTIC(stallThreshold);
  ADD_DIAGNOSTIC(coolConfig);
  ADD_DIAGNOSTIC(microstepCurrents);
  ADD_DIAGNOSTIC(chopperConfig);
  ADD_DIAGNOSTIC(pwmConfig);
  ADD_DIAGNOSTIC(pwmScale);
  ADD_DIAGNOSTIC(pwmAuto);
#undef ADD_DIAGNOSTIC
}

void addCommonState(JsonDocument& document) {
  document["tmcHealthy"] = tmc.healthy();
  document["tmcFault"] = tmc.faultLatched();
  document["ip"] =
      WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("offline");
  addMotionState(document);
  addServoState(document);
  JsonObject limits = document["limits"].to<JsonObject>();
  limits["maxSpeed"] = motionMaxSpeed;
  limits["maxAcceleration"] = motionMaxAcceleration;
}

void sendSnapshot(uint32_t clientId) {
  JsonDocument document;
  document["v"] = 1;
  document["type"] = "snapshot";
  addCommonState(document);
  addDriverSettings(document);
  addDiagnostics(document);
  sendDocument(clientId, document);
}

void broadcastSnapshot() {
  JsonDocument document;
  document["v"] = 1;
  document["type"] = "snapshot";
  addCommonState(document);
  addDriverSettings(document);
  addDiagnostics(document);
  broadcastDocument(document);
}

void broadcastTelemetry() {
  JsonDocument document;
  document["v"] = 1;
  document["type"] = "telemetry";
  addCommonState(document);
  broadcastDocument(document);
}

void broadcastDiagnostics() {
  JsonDocument document;
  document["v"] = 1;
  document["type"] = "diagnostics";
  document["tmcHealthy"] = tmc.healthy();
  document["tmcFault"] = tmc.faultLatched();
  addDiagnostics(document);
  broadcastDocument(document);
}

bool parseMotion(JsonObjectConst payload, uint32_t& speed, uint32_t& acceleration) {
  if (!payload["speed"].is<uint32_t>() ||
      !payload["acceleration"].is<uint32_t>()) return false;
  speed = payload["speed"].as<uint32_t>();
  acceleration = payload["acceleration"].as<uint32_t>();
  return speed >= 1 && speed <= motionMaxSpeed &&
         speed <= AppConfig::kMaxStepRate && acceleration >= 1 &&
         acceleration <= motionMaxAcceleration &&
         acceleration <= AppConfig::kMaxAcceleration;
}

bool parseDriverSettings(JsonObjectConst payload, DriverSettings& value) {
#define READ_U8(name)                                      \
  if (!payload[#name].is<uint8_t>()) return false;         \
  value.name = payload[#name].as<uint8_t>()
#define READ_U16(name)                                     \
  if (!payload[#name].is<uint16_t>()) return false;        \
  value.name = payload[#name].as<uint16_t>()
#define READ_U32(name)                                     \
  if (!payload[#name].is<uint32_t>()) return false;        \
  value.name = payload[#name].as<uint32_t>()
#define READ_BOOL(name)                                    \
  if (!payload[#name].is<bool>()) return false;            \
  value.name = payload[#name].as<bool>()
  READ_U16(runCurrentMa);
  READ_U8(holdPercent);
  READ_U8(holdDelay);
  READ_U8(powerDownDelay);
  READ_U16(microsteps);
  READ_BOOL(interpolation);
  READ_BOOL(doubleEdge);
  READ_BOOL(multistepFilter);
  READ_BOOL(spreadCycle);
  READ_BOOL(shaftInverted);
  READ_U32(stealthThreshold);
  READ_U8(offTime);
  READ_U8(blankTime);
  if (!payload["hysteresisEnd"].is<int8_t>()) return false;
  value.hysteresisEnd = payload["hysteresisEnd"].as<int8_t>();
  READ_U8(hysteresisStart);
  READ_U8(pwmOffset);
  READ_U8(pwmGradient);
  READ_U8(pwmFrequency);
  READ_BOOL(pwmAutoscale);
  READ_BOOL(pwmAutograd);
  READ_U8(freewheel);
  READ_U8(pwmRegulation);
  READ_U8(pwmLimit);
  READ_U32(coolThreshold);
  READ_U8(stallThreshold);
  READ_U8(coolMinimum);
  READ_U8(coolUpStep);
  READ_U8(coolMaximum);
  READ_U8(coolDownStep);
  READ_BOOL(coolMinimumQuarter);
  READ_U8(uartReplyDelay);
#undef READ_U8
#undef READ_U16
#undef READ_U32
#undef READ_BOOL
  return DriverSettings::validate(value);
}

void emergencyStop() {
  cycle.stop(motion);
  motion.emergencyStop();
  servo.detach();
  jogOwner = 0;
  sweepOwner = 0;
}

bool requireHealthyOrError(uint32_t clientId, const String& requestId) {
  if (!tmc.healthy()) {
    sendError(clientId, requestId, "tmc_offline",
              "TMC2209 UART is not healthy; retry UART before moving");
    return false;
  }
  return true;
}

void handleCommand(const NetworkEvent& event) {
  JsonDocument document;
  const auto error = deserializeJson(document, event.json);
  if (error) {
    sendError(event.clientId, "", "invalid_json", error.c_str());
    return;
  }
  const String requestId = document["id"] | "";
  const char* type = document["type"] | "";
  if ((document["v"] | 0) != 1 || requestId.isEmpty() || type[0] == '\0' ||
      !document["payload"].is<JsonObject>()) {
    sendError(event.clientId, requestId, "invalid_envelope",
              "Expected protocol v1 command with id, type, and payload");
    return;
  }
  const JsonObjectConst payload = document["payload"].as<JsonObjectConst>();
  bool accepted = true;

  if (strcmp(type, "enable") == 0) {
    if (!requireHealthyOrError(event.clientId, requestId)) return;
    motion.enable();
  } else if (strcmp(type, "disable") == 0) {
    cycle.stop(motion);
    motion.disable();
  } else if (strcmp(type, "emergency") == 0) {
    emergencyStop();
  } else if (strcmp(type, "stop") == 0) {
    cycle.stop(motion);
    motion.stop();
    jogOwner = 0;
  } else if (strcmp(type, "jog") == 0) {
    uint32_t speed = 0, acceleration = 0;
    int8_t direction = 0;
    if (!requireHealthyOrError(event.clientId, requestId)) return;
    if (!payload["direction"].is<int8_t>() ||
        !parseMotion(payload, speed, acceleration)) accepted = false;
    if (accepted) {
      direction = payload["direction"].as<int8_t>();
      accepted = (direction == -1 || direction == 1) &&
                 motion.jog(direction, speed, acceleration);
      if (accepted) jogOwner = event.clientId;
    }
  } else if (strcmp(type, "moveBy") == 0 || strcmp(type, "moveTo") == 0) {
    uint32_t speed = 0, acceleration = 0;
    if (!requireHealthyOrError(event.clientId, requestId)) return;
    if (!parseMotion(payload, speed, acceleration)) accepted = false;
    if (accepted && strcmp(type, "moveBy") == 0) {
      if (!payload["steps"].is<int64_t>()) accepted = false;
      else accepted = motion.moveBy(payload["steps"].as<int64_t>(), speed, acceleration);
    } else if (accepted) {
      if (!payload["target"].is<int64_t>()) accepted = false;
      else accepted = motion.moveTo(payload["target"].as<int64_t>(), speed, acceleration);
    }
    if (accepted) {
      cycle.stop(motion);
      jogOwner = 0;
    }
  } else if (strcmp(type, "zero") == 0) {
    accepted = motion.zero();
  } else if (strcmp(type, "invertDirection") == 0) {
    if (!motion.idle() || !payload["inverted"].is<bool>()) accepted = false;
    else {
      motion.setDirectionInverted(payload["inverted"].as<bool>());
      motion.invalidateReference();
    }
  } else if (strcmp(type, "cycleStart") == 0) {
    uint32_t speed = 0, acceleration = 0;
    if (!requireHealthyOrError(event.clientId, requestId)) return;
    if (!parseMotion(payload, speed, acceleration) ||
        !payload["travel"].is<uint32_t>() || !payload["dwellMs"].is<uint32_t>() ||
        !payload["repeats"].is<uint32_t>() || !payload["direction"].is<int8_t>() ||
        !payload["infinite"].is<bool>()) accepted = false;
    if (accepted) {
      const uint32_t travel = payload["travel"].as<uint32_t>();
      const uint32_t dwell = payload["dwellMs"].as<uint32_t>();
      accepted = travel <= AppConfig::kMaxMoveSteps && dwell <= AppConfig::kMaxDwellMs &&
                 cycle.start(motion, travel, payload["direction"].as<int8_t>(), speed,
                             acceleration, dwell, payload["repeats"].as<uint32_t>(),
                             payload["infinite"].as<bool>(), millis());
    }
  } else if (strcmp(type, "cycleStop") == 0) {
    cycle.stop(motion, true);
  } else if (strcmp(type, "motionLimits") == 0) {
    if (!motion.idle() || !payload["maxSpeed"].is<uint32_t>() ||
        !payload["maxAcceleration"].is<uint32_t>()) {
      accepted = false;
    } else {
      const uint32_t speed = payload["maxSpeed"].as<uint32_t>();
      const uint32_t acceleration = payload["maxAcceleration"].as<uint32_t>();
      accepted = speed >= 1 && speed <= AppConfig::kMaxStepRate &&
                 acceleration >= 1 && acceleration <= AppConfig::kMaxAcceleration;
      if (accepted) {
        motionMaxSpeed = speed;
        motionMaxAcceleration = acceleration;
      }
    }
  } else if (strcmp(type, "driverApply") == 0) {
    DriverSettings updated = tmc.settings();
    if (!motion.idle() || cycle.active()) {
      sendError(event.clientId, requestId, "motor_busy",
                "Stop the motor before changing TMC2209 settings");
      return;
    }
    if (!parseDriverSettings(payload, updated)) accepted = false;
    if (accepted) {
      const bool invalidate = updated.changesStepInterpretation(tmc.settings());
      motion.disable(invalidate);
      accepted = tmc.applySettings(updated) && tmc.pollDiagnostics();
    }
  } else if (strcmp(type, "tmcRetry") == 0) {
    motion.disable();
    accepted = tmc.retry();
    handledTmcFault = false;
  } else if (strcmp(type, "diagnostics") == 0) {
    accepted = tmc.pollDiagnostics();
  } else if (strcmp(type, "clearStatus") == 0) {
    tmc.clearStatus();
  } else if (strcmp(type, "servoAttach") == 0) {
    accepted = servo.attach();
  } else if (strcmp(type, "servoDetach") == 0) {
    servo.detach();
    sweepOwner = 0;
  } else if (strcmp(type, "servoRange") == 0) {
    if (!payload["minimumUs"].is<uint16_t>() ||
        !payload["maximumUs"].is<uint16_t>()) accepted = false;
    else accepted = servo.setRange(payload["minimumUs"].as<uint16_t>(),
                                   payload["maximumUs"].as<uint16_t>());
  } else if (strcmp(type, "servoAngle") == 0) {
    if (!payload["angle"].is<uint16_t>()) accepted = false;
    else accepted = servo.writeAngle(payload["angle"].as<uint16_t>());
  } else if (strcmp(type, "servoPulse") == 0) {
    if (!payload["pulseUs"].is<uint16_t>()) accepted = false;
    else accepted = servo.writePulse(payload["pulseUs"].as<uint16_t>());
  } else if (strcmp(type, "servoSweep") == 0) {
    ServoSweep::Settings settings;
    if (!payload["minimumUs"].is<uint16_t>() ||
        !payload["maximumUs"].is<uint16_t>() ||
        !payload["startUs"].is<uint16_t>() || !payload["endUs"].is<uint16_t>() ||
        !payload["travelMs"].is<uint32_t>() || !payload["dwellMs"].is<uint32_t>() ||
        !payload["repeats"].is<uint32_t>() || !payload["infinite"].is<bool>()) {
      accepted = false;
    } else {
      settings.minimumUs = payload["minimumUs"].as<uint16_t>();
      settings.maximumUs = payload["maximumUs"].as<uint16_t>();
      settings.startUs = payload["startUs"].as<uint16_t>();
      settings.endUs = payload["endUs"].as<uint16_t>();
      settings.travelMs = payload["travelMs"].as<uint32_t>();
      settings.dwellMs = payload["dwellMs"].as<uint32_t>();
      settings.repeats = payload["repeats"].as<uint32_t>();
      settings.infinite = payload["infinite"].as<bool>();
      accepted = servo.startSweep(settings, millis());
      if (accepted) sweepOwner = event.clientId;
    }
  } else if (strcmp(type, "servoSweepStop") == 0) {
    servo.stopSweep(true);
    sweepOwner = 0;
  } else {
    sendError(event.clientId, requestId, "unknown_command", "Unknown command type");
    return;
  }

  if (!accepted) {
    sendError(event.clientId, requestId, "invalid_command",
              "Command values or current controller state are invalid");
    return;
  }
  sendAck(event.clientId, requestId);
  broadcastSnapshot();
}

void enqueueEvent(NetworkEventType type, uint32_t clientId) {
  if (commandQueue == nullptr) return;
  NetworkEvent event;
  event.type = type;
  event.clientId = clientId;
  xQueueSend(commandQueue, &event, 0);
}

void onWebSocketEvent(AsyncWebSocket*, AsyncWebSocketClient* client,
                      AwsEventType type, void* argument, uint8_t* data,
                      size_t length) {
  if (type == WS_EVT_CONNECT) {
    enqueueEvent(NetworkEventType::Connected, client->id());
    return;
  }
  if (type == WS_EVT_DISCONNECT) {
    enqueueEvent(NetworkEventType::Disconnected, client->id());
    return;
  }
  if (type != WS_EVT_DATA) return;
  auto* frame = static_cast<AwsFrameInfo*>(argument);
  if (!frame->final || frame->index != 0 || frame->len != length ||
      frame->opcode != WS_TEXT || length == 0 || length >= kCommandBytes) {
    client->text("{\"v\":1,\"type\":\"error\",\"code\":\"invalid_frame\","
                 "\"message\":\"Command must be one text frame under 3072 bytes\"}");
    return;
  }
  NetworkEvent event;
  event.type = NetworkEventType::Message;
  event.clientId = client->id();
  memcpy(event.json, data, length);
  event.json[length] = '\0';
  if (xQueueSend(commandQueue, &event, 0) != pdTRUE) {
    client->text("{\"v\":1,\"type\":\"error\",\"code\":\"queue_full\","
                 "\"message\":\"Controller is busy; retry\"}");
  }
}

void processNetworkEvents() {
  NetworkEvent event;
  while (xQueueReceive(commandQueue, &event, 0) == pdTRUE) {
    if (event.type == NetworkEventType::Connected) {
      sendSnapshot(event.clientId);
    } else if (event.type == NetworkEventType::Disconnected) {
      if (jogOwner == event.clientId) {
        motion.stop();
        jogOwner = 0;
      }
      if (sweepOwner == event.clientId && servo.sweep().active() &&
          servo.sweep().infinite()) {
        servo.detach();
        sweepOwner = 0;
      }
    } else {
      handleCommand(event);
    }
  }
}

void updateWifi(uint32_t now) {
  if (WiFi.status() == WL_CONNECTED) {
    if (!mdnsStarted) {
      mdnsStarted = MDNS.begin("blinds-test");
      if (mdnsStarted) {
        MDNS.addService("http", "tcp", 80);
        Serial.printf("Wi-Fi connected: http://%s/ (blinds-test.local)\n",
                      WiFi.localIP().toString().c_str());
      }
    }
    return;
  }
  if (mdnsStarted) {
    MDNS.end();
    mdnsStarted = false;
  }
  if (WIFI_SSID[0] == '\0') return;
  if (lastWifiAttemptAt == 0 ||
      now - lastWifiAttemptAt >= AppConfig::kWifiRetryMs) {
    lastWifiAttemptAt = now;
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("Connecting to Wi-Fi SSID %s\n", WIFI_SSID);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(50);
  servo.detach();
  if (!motion.begin()) Serial.println("Motion timer initialization failed");
  motion.disable();
  tmc.begin();

  commandQueue = xQueueCreate(kCommandQueueLength, sizeof(NetworkEvent));

  WiFi.mode(WIFI_STA);
  WiFi.setHostname("blinds-test");
  WiFi.setAutoReconnect(true);
  updateWifi(millis());

  websocket.onEvent(onWebSocketEvent);
  server.addHandler(&websocket);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/html", WEB_UI);
  });
  server.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "text/plain", "Not found");
  });
  server.begin();
}

void loop() {
  const uint32_t now = millis();
  processNetworkEvents();
  cycle.update(motion, now);
  servo.update(now);
  updateWifi(now);

  if (now - lastDiagnosticsAt >= AppConfig::kDiagnosticsPeriodMs) {
    lastDiagnosticsAt = now;
    tmc.pollDiagnostics();
    if (tmc.faultLatched() && !handledTmcFault) {
      cycle.stop(motion);
      motion.emergencyStop();
      handledTmcFault = true;
      broadcastEvent("TMC2209 UART failed three times; motor disabled");
    }
    broadcastDiagnostics();
  }
  if (now - lastTelemetryAt >= AppConfig::kTelemetryPeriodMs) {
    lastTelemetryAt = now;
    broadcastTelemetry();
  }
  websocket.cleanupClients();
}
