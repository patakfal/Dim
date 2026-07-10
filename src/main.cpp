// Main application controlling ESP8266 based mains-sensing dimmer. Handles
// physical input timing, brightness persistence, and MQTT state reporting.
#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <array>
#include <cstring>

#include "DimNetwork.h"
#include "SysLogger.h"

// Hardware wiring: D7 drives the triac gate (active low), D6 senses mains via opto.
const uint8_t ledPin = D7;
const uint8_t sensePin = D6;
constexpr uint16_t pwmMax = 1023;
const uint16_t fadeDelayMs = 500;
const uint32_t toggleWindowMs = 1000;
const uint32_t pwmFrequency = 1000;

/**
 * @brief Converts a 0-100 brightness percentage into a PWM raw value.
 *
 * Uses integer math rounded to the nearest integer to keep calculations fast
 * on the ESP8266 while still matching the perceived brightness curve.
 */
constexpr uint16_t percentToRaw(uint8_t percent) {
  return static_cast<uint16_t>((static_cast<uint32_t>(percent) * pwmMax + 50U) / 100U);
}

/**
 * @brief Scales a PWM raw value back to a whole percentage.
 *
 * This matches percentToRaw so round-tripping produces stable numbers for
 * MQTT/state reporting.
 */
constexpr uint8_t rawToPercent(uint16_t raw) {
  return static_cast<uint8_t>((static_cast<uint32_t>(raw) * 100U + pwmMax / 2U) / pwmMax);
}

/** Lookup table representing a human-friendly discrete brightness curve. */
constexpr std::array<uint8_t, 20> brightnessStepPercents{
    10, 12, 14, 16, 18, 21, 24, 27, 30, 33,
    36, 40, 45, 50, 55, 60, 67, 75, 85, 100};

template <size_t N>
/**
 * @brief Builds the raw PWM equivalents for the configured brightness steps.
 */
constexpr std::array<uint16_t, N> computeBrightnessStepRaws(const std::array<uint8_t, N> &percents) {
  std::array<uint16_t, N> raws{};
  for (size_t i = 0; i < N; ++i) {
    raws[i] = percentToRaw(percents[i]);
  }
  return raws;
}

constexpr auto brightnessStepRaws = computeBrightnessStepRaws(brightnessStepPercents);
constexpr size_t brightnessStepCount = brightnessStepPercents.size();
constexpr int fadeIndexStep = 1;

/** @brief Returns the dimmest raw value we allow for smooth fades. */
constexpr uint16_t minBrightnessRaw() {
  return brightnessStepRaws[0];
}

/** @brief Returns the brightest raw value available in the lookup table. */
constexpr uint16_t maxBrightnessRaw() {
  return brightnessStepRaws[brightnessStepCount - 1];
}

const size_t eepromSize = 8;
const uint8_t eepromMagic = 0xA5;
const uint16_t eepromMagicAddr = 0;
const uint16_t eepromBrightnessAddr = 1;

const char *wifiSsid = "lynx_homeK_24";
const char *wifiPassword = "Lartubu1!4!7!8!";
const uint32_t wifiConnectTimeoutMs = 15000;
const unsigned long wifiRetryIntervalMs = 10000;

const char *mqttHost = "192.168.50.10";
const uint16_t mqttPort = 1883;
const char *mqttClientId = "dim-controller";
const char *mqttUsername = "lynx";
const char *mqttPassword = "679ZMtgmzGWqknZ";
const IPAddress syslogServer(192, 168, 50, 170);
const uint16_t syslogPort = 514;
bool otaReady = false;

Mode currentMode = Mode::Hold;
// Runtime state mirrored to MQTT.
uint16_t brightness = 0;
int brightnessIndex = 0;
int direction = fadeIndexStep;
int lastSenseState = LOW;
unsigned long lastSenseChangeMs = 0;
unsigned long senseInactiveStartMs = 0;
unsigned long lastBrightnessUpdateMs = 0;
unsigned long suppressFadeUntilMs = 0;
uint16_t lastSavedBrightness = 0;
unsigned long lastQuickToggleMs = 0;
uint8_t quickToggleCount = 0;

/**
 * @brief Samples the mains sense input during boot and returns the majority vote.
 *
 * GPIOs can float while the ESP8266 boots, so taking multiple readings avoids
 * entering the wrong mode before the inputs settle.
 */
int readInitialSenseState() {
  int lowCount = 0;
  int highCount = 0;
  for (int i = 0; i < 12; ++i) {
    const int sample = digitalRead(sensePin);
    if (sample == LOW) {
      ++lowCount;
    } else {
      ++highCount;
    }
    delay(1);
  }
  return (lowCount >= highCount) ? LOW : HIGH;
}

/**
 * @brief Quantizes a raw PWM value to the closest predefined brightness step.
 *
 * Keeps fades predictable by ensuring indices always map to the discrete
 * brightnessStepRaws table rather than arbitrary PWM values.
 */
int quantizeToStepIndex(int value) {
  const int firstRaw = static_cast<int>(brightnessStepRaws[0]);
  if (value <= firstRaw) {
    return 0;
  }
  const int lastRaw = static_cast<int>(brightnessStepRaws[brightnessStepCount - 1]);
  if (value >= lastRaw) {
    return static_cast<int>(brightnessStepCount) - 1;
  }
  for (size_t i = 0; i < brightnessStepCount - 1; ++i) {
    const int currentRaw = static_cast<int>(brightnessStepRaws[i]);
    const int nextRaw = static_cast<int>(brightnessStepRaws[i + 1]);
    const int midpoint = (currentRaw + nextRaw) / 2;
    if (value <= midpoint) {
      return static_cast<int>(i);
    }
  }
  return static_cast<int>(brightnessStepCount) - 1;
}

/**
 * @brief Writes a raw PWM value to the LED and optionally updates bookkeeping.
 *
 * Centralizes LED writes so brightness, indices, and MQTT notifications remain
 * consistent across manual, MQTT, and fade-driven changes.
 */
void applyBrightnessValue(uint16_t value, bool updateIndex = true) {
  value = constrain(value, static_cast<uint16_t>(0), pwmMax);
  brightness = value;
  if (updateIndex) {
    brightnessIndex = quantizeToStepIndex(value);
  }
  const uint16_t pwmValue = pwmMax - brightness; // D7 LED is active-low
  analogWrite(ledPin, pwmValue);
  networkNotifyBrightnessChange();
}

/**
 * @brief Writes a brightness step index after clamping to valid bounds.
 *
 * Used whenever fades or MQTT commands reference discrete steps so the raw PWM
 * value always matches the lookup table entry.
 */
void applyBrightnessIndex(int index) {
  index = constrain(index, 0, static_cast<int>(brightnessStepCount) - 1);
  brightnessIndex = index;
  const uint16_t stepped = brightnessStepRaws[index];
  applyBrightnessValue(stepped, false);
}

/** @brief Helper exposed to networking so it can print friendly mode names. */
const char *modeName(Mode mode) {
  return mode == Mode::Fade ? "fade" : "hold";
}

/**
 * @brief Emits a concise serial log entry describing the current controller state.
 *
 * The logs make it easy to correlate sense toggles, fades, and MQTT commands
 * when debugging timing issues.
 */
void reportStatus(const char *source) {
  const bool mainsPresent = mainsPresentFromSense(lastSenseState);
  const bool isOn = mainsPresent && brightness > 0;
  SysLogger::logf("%s mode=%s light=%s brightness=%u%%",
                  source,
                  modeName(currentMode),
                  isOn ? "ON" : "OFF",
                  rawToPercent(brightness));
}

/**
 * @brief Loads the last persisted brightness from EEPROM.
 *
 * Uses a magic byte to validate the stored data and falls back to the minimum
 * brightness when the signature is missing or corrupted.
 */
uint16_t loadBrightnessFromEEPROM() {
  if (EEPROM.read(eepromMagicAddr) == eepromMagic) {
    uint16_t stored = 0;
    EEPROM.get(eepromBrightnessAddr, stored);
    if (stored <= pwmMax) {
      lastSavedBrightness = stored;
      SysLogger::logf("eeprom_load=%u%%", rawToPercent(stored));
      return stored;
    }
  }
  lastSavedBrightness = minBrightnessRaw();
  SysLogger::log("eeprom_load=default");
  return lastSavedBrightness;
}

/**
 * @brief Writes the active brightness to EEPROM when it changes.
 *
 * Skips redundant commits so we do not burn through the limited write cycles.
 */
void persistBrightness() {
  const uint16_t value = brightness;
  if (value == lastSavedBrightness && EEPROM.read(eepromMagicAddr) == eepromMagic) {
    return; // nothing changed
  }

  EEPROM.write(eepromMagicAddr, eepromMagic);
  EEPROM.put(eepromBrightnessAddr, value);
  EEPROM.commit();
  lastSavedBrightness = value;
  SysLogger::logf("eeprom_save=%u%%", rawToPercent(value));
}

/**
 * @brief Arms the controller for automatic fading starting from the current step.
 *
 * Initializes the fade direction, synchronizes the step index, and clears any
 * quick-toggle counters so subsequent toggles behave predictably.
 */
void enterFade(const char *reason) {
  if (currentMode == Mode::Fade) {
    return;
  }
  const int startIndex = quantizeToStepIndex(brightness);
  applyBrightnessIndex(startIndex);
  currentMode = Mode::Fade;
  quickToggleCount = 0;
  suppressFadeUntilMs = 0;
  if (brightnessIndex <= 0) {
    direction = fadeIndexStep;
  } else if (brightnessIndex >= static_cast<int>(brightnessStepCount) - 1) {
    direction = -fadeIndexStep;
  } else if (direction == 0) {
    direction = fadeIndexStep;
  }
  lastBrightnessUpdateMs = millis();
  reportStatus(reason);
}

/**
 * @brief Leaves fade mode, persists the resulting brightness, and debounces.
 *
 * Applies a temporary suppression window so accidental mains jitters do not
 * immediately restart a fade.
 */
void commitFade(const char *reason) {
  if (currentMode != Mode::Fade) {
    return;
  }
  currentMode = Mode::Hold;
  quickToggleCount = 0;
  persistBrightness();
  suppressFadeUntilMs = millis() + toggleWindowMs;
  reportStatus(reason);
}

/**
 * @brief Returns whether a new fade is allowed based on the suppression timer.
 */
bool canStartFade(unsigned long now) {
  if (suppressFadeUntilMs == 0) {
    return true;
  }
  const long delta = static_cast<long>(now - suppressFadeUntilMs);
  if (delta >= 0) {
    suppressFadeUntilMs = 0;
    return true;
  }
  return false;
}

/**
 * @brief Applies a brightness value, persists it, and emits status.
 *
 * MQTT requests bypass the step quantization so Home Assistant can drive exact
 * raw values while manual events snap to the discrete curve.
 */
void holdAndPersist(int value, const char *reason) {
  currentMode = Mode::Hold;
  const bool isMqttReason = (reason != nullptr && strncmp(reason, "mqtt_", 5) == 0);
  if (isMqttReason) {
    applyBrightnessValue(static_cast<uint16_t>(value), false);
  } else {
    const int index = quantizeToStepIndex(value);
    applyBrightnessIndex(index);
  }
  persistBrightness();
  suppressFadeUntilMs = millis() + toggleWindowMs;
  quickToggleCount = 0;
  reportStatus(reason);
}

/** @brief Exposes the current mode to the networking layer. */
Mode getCurrentMode() {
  return currentMode;
}

/** @brief Returns the raw brightness mirrored to MQTT. */
uint16_t getCurrentBrightness() {
  return brightness;
}

/** @brief Returns the most recent mains sense reading. */
int getCurrentSenseState() {
  return lastSenseState;
}

/**
 * @brief Interprets quick mains toggles as gestures (double-click for max, single for fade).
 *
 * Maintains a sliding window counter so noise on the mains sense input cannot
 * spam mode changes.
 */
void handleQuickToggle(unsigned long now) {
  if (now - lastQuickToggleMs <= toggleWindowMs) {
    quickToggleCount++;
  } else {
    quickToggleCount = 1;
  }
  lastQuickToggleMs = now;

  if (quickToggleCount >= 2) {
    quickToggleCount = 0;
    holdAndPersist(maxBrightnessRaw(), "double_click_full");
  } else if (canStartFade(now)) {
    enterFade("start_fade_quick_toggle");
  } else {
    reportStatus("hold_quick_toggle_suppressed");
  }
}

/**
 * @brief Initializes hardware, networking, and restores persisted brightness.
 *
 * Sets up PWM, GPIO, status callbacks, and synchronizes the network layer with
 * the current state before entering the main loop.
 */
void setup() {
  delay(200); // let it boot cleanly
  SysLogger::begin(syslogServer, syslogPort, mqttClientId, mqttClientId);
  analogWriteFreq(pwmFrequency);
  analogWriteRange(pwmMax);
  pinMode(ledPin, OUTPUT);
  pinMode(sensePin, INPUT);
  delay(10); // allow the sense line to settle before sampling
  EEPROM.begin(eepromSize);

  // Capture baseline mains state so mode logic starts deterministic.
  lastSenseState = readInitialSenseState();
  lastSenseChangeMs = millis();
  senseInactiveStartMs = mainsPresentFromSense(lastSenseState) ? 0 : lastSenseChangeMs;

  // Provide the networking layer with live state callbacks and begin Wi-Fi/MQTT.
  NetworkStatusView view{
    getCurrentMode,
    getCurrentBrightness,
    getCurrentSenseState,
    modeName
  };
  networkInit(wifiSsid,
              wifiPassword,
              wifiConnectTimeoutMs,
              wifiRetryIntervalMs,
              view,
              holdAndPersist,
              mqttHost,
              mqttPort,
              mqttClientId,
              mqttUsername,
              mqttPassword);

  // Restore persisted brightness and notify the network if the sense input changed.
  const uint16_t stored = loadBrightnessFromEEPROM();
  applyBrightnessValue(stored);
  lastSavedBrightness = brightness;
  const int confirmedSense = readInitialSenseState();
  if (confirmedSense != lastSenseState) {
    lastSenseState = confirmedSense;
    lastSenseChangeMs = millis();
    senseInactiveStartMs = mainsPresentFromSense(lastSenseState) ? 0 : lastSenseChangeMs;
    networkNotifyBrightnessChange();
  }
  reportStatus("boot");
}

/** @brief Lazily starts OTA once Wi-Fi is up so uploads can happen without serial. */
void ensureOtaReady() {
  if (otaReady || !networkIsConnected()) {
    return;
  }
  ArduinoOTA.setHostname(mqttClientId);
  ArduinoOTA.onStart([]() { SysLogger::log("ota_start"); });
  ArduinoOTA.onEnd([]() { SysLogger::log("ota_end"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    const unsigned int percent = (progress * 100U) / total;
    SysLogger::logf("ota_progress=%u%%", percent);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    SysLogger::logf("ota_error=%u", static_cast<unsigned>(error));
  });
  ArduinoOTA.begin();
  otaReady = true;
  SysLogger::log("ota_ready");
}

/**
 * @brief Cooperative scheduler that processes networking, sense changes, and fades.
 *
 * Must run frequently to keep MQTT responsive and ensure fades stay smooth. Each
 * iteration performs three jobs:
 *   1. Pump the networking layer so Wi-Fi/MQTT callbacks execute.
 *   2. Sample the mains sense input and translate toggles into mode/brightness changes.
 *   3. Advance fade animations or auto-commit them when mains stay idle.
 */
void loop() {
  networkLoop();
  ensureOtaReady();
  if (otaReady && networkIsConnected()) {
    ArduinoOTA.handle();
  } else if (otaReady && !networkIsConnected()) {
    otaReady = false;
  }

  // Continuously read mains sense input to detect human gestures.
  const unsigned long now = millis();
  const int senseState = digitalRead(sensePin);

  if (senseState != lastSenseState) {
    const unsigned long delta = now - lastSenseChangeMs;
    const bool quickToggle = delta <= toggleWindowMs; // taps closer than toggleWindowMs are treated as gestures
    bool shouldReportHold = false; // defer logging until we know if we stayed in Hold

    if (currentMode == Mode::Fade) {
      if (quickToggle) {
        holdAndPersist(maxBrightnessRaw(), "double_click_full");
      } else {
        commitFade("commit_long_toggle");
      }
    } else if (quickToggle) {
      handleQuickToggle(now);
    } else {
      quickToggleCount = 0;
      suppressFadeUntilMs = 0;
      shouldReportHold = true;
    }

    // Push updated sense + brightness to MQTT so HA mirrors the physical change.
    lastSenseChangeMs = now;
    lastSenseState = senseState;
    senseInactiveStartMs = mainsPresentFromSense(senseState) ? 0 : now;
    networkNotifyBrightnessChange();
    if (shouldReportHold) {
      reportStatus("hold_toggle");
    }
  } else if (!mainsPresentFromSense(senseState) && senseInactiveStartMs == 0) {
    senseInactiveStartMs = now; // remember when mains went inactive to timeout fades later
  }

  // Drive fade animation when active; auto-commit if mains stay low.
  if (currentMode == Mode::Fade) {
    if (senseInactiveStartMs != 0 && (now - senseInactiveStartMs) > toggleWindowMs) {
      quickToggleCount = 0;
      commitFade("commit_low_timeout");
    } else if ((now - lastBrightnessUpdateMs) >= fadeDelayMs) {
      int nextIndex = brightnessIndex + direction;
      if (nextIndex <= 0 || nextIndex >= static_cast<int>(brightnessStepCount) - 1) {
        nextIndex = constrain(nextIndex, 0, static_cast<int>(brightnessStepCount) - 1);
        direction = -direction;
      }
      applyBrightnessIndex(nextIndex);
      lastBrightnessUpdateMs = now;
      reportStatus("fade");
    }
  }

  delay(5);
}
