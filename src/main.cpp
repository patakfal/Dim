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

// Hardware wiring: D7 drives the triac gate (active low), D6 senses mains via opto.
const uint8_t ledPin = D7;
const uint8_t sensePin = D6;
constexpr uint16_t pwmMax = 1023;
const uint32_t toggleWindowMs = 1000;
const uint32_t lowBrightnessToggleWindowMs = 1500;
uint32_t currentToggleWindowMs();
const uint32_t senseDebounceMs = 100;
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

/** Lookup table for discrete brightness steps cycled by mains toggles. */
constexpr std::array<uint8_t, 3> brightnessStepPercents{10, 50, 100};

template <size_t N>
constexpr std::array<uint16_t, N> computeBrightnessStepRaws(const std::array<uint8_t, N> &percents) {
  std::array<uint16_t, N> raws{};
  for (size_t i = 0; i < N; ++i) {
    raws[i] = percentToRaw(percents[i]);
  }
  return raws;
}

constexpr auto brightnessStepRaws = computeBrightnessStepRaws(brightnessStepPercents);
constexpr size_t brightnessStepCount = brightnessStepPercents.size();

/** @brief Returns the dimmest raw value we allow. */
constexpr uint16_t minBrightnessRaw() {
  return brightnessStepRaws[0];
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
bool otaReady = false;

Mode currentMode = Mode::Hold;
// Runtime state mirrored to MQTT.
uint16_t brightness = 0;
bool forcedOffDueToLow = false;
uint16_t forcedOffPreviousBrightness = 0;
int lastSenseState = LOW;
unsigned long lastSenseChangeMs = 0;
uint16_t lastSavedBrightness = 0;

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
 * Keeps hardware/manual changes aligned to the discrete brightnessStepRaws table.
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
 * @brief Writes a raw PWM value to the LED.
 *
 * Centralizes LED writes so brightness and MQTT notifications remain consistent
 * across manual, MQTT, and toggle-driven changes.
 */
void applyBrightnessValue(uint16_t value) {
  value = constrain(value, static_cast<uint16_t>(0), pwmMax);
  brightness = value;
  const uint16_t pwmValue = pwmMax - brightness; // D7 LED is active-low
  analogWrite(ledPin, pwmValue);
  networkNotifyBrightnessChange();
}

/**
 * @brief Writes a brightness step index after clamping to valid bounds.
 */
void applyBrightnessIndex(int index) {
  index = constrain(index, 0, static_cast<int>(brightnessStepCount) - 1);
  const uint16_t stepped = brightnessStepRaws[index];
  applyBrightnessValue(stepped);
}

/** @brief Helper exposed to networking so it can print friendly mode names. */
const char *modeName(Mode mode) {
  return mode == Mode::Fade ? "fade" : "hold";
}

/**
 * @brief Emits a concise serial log entry describing the current controller state.
 */
void reportStatus(const char *source) {
  (void)source;
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
      return stored;
    }
  }
  lastSavedBrightness = minBrightnessRaw();
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
    applyBrightnessValue(static_cast<uint16_t>(value));
  } else {
    const int index = quantizeToStepIndex(value);
    applyBrightnessIndex(index);
  }
  persistBrightness();
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

/** @brief Computes the next step index when cycling via a mains toggle. */
int nextStepIndex(uint8_t currentPercent) {
  for (size_t i = 0; i < brightnessStepCount; ++i) {
    const uint8_t step = brightnessStepPercents[i];
    if (currentPercent < step) {
      return static_cast<int>(i);
    }
    if (currentPercent == step) {
      return static_cast<int>((i + 1) % brightnessStepCount);
    }
  }
  return 0;
}

/**
 * @brief Maps quick mains toggles to the next discrete brightness step (10/50/100%).
 */
void handleQuickToggle(unsigned long now) {
  (void)now;
  uint8_t basePercent = rawToPercent(brightness);
  if (forcedOffDueToLow && forcedOffPreviousBrightness > 0) {
    // Use the pre-forced-off level so toggles advance from the last real brightness.
    basePercent = rawToPercent(forcedOffPreviousBrightness);
    forcedOffDueToLow = false;
  }
  const int nextIndex = nextStepIndex(basePercent);
  applyBrightnessIndex(nextIndex);
  persistBrightness();
  reportStatus("step_toggle");
}

/**
 * @brief Initializes hardware, networking, and restores persisted brightness.
 *
 * Sets up PWM, GPIO, status callbacks, and synchronizes the network layer with
 * the current state before entering the main loop.
 */
void setup() {
  delay(200); // let it boot cleanly
  analogWriteFreq(pwmFrequency);
  analogWriteRange(pwmMax);
  pinMode(ledPin, OUTPUT);
  pinMode(sensePin, INPUT);
  delay(10); // allow the sense line to settle before sampling
  EEPROM.begin(eepromSize);

  // Capture baseline mains state so mode logic starts deterministic.
  lastSenseState = readInitialSenseState();
  lastSenseChangeMs = millis();

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
  ArduinoOTA.begin();
  otaReady = true;
  networkSendSyslog("<134>dim-controller ota_ready");
}

/**
 * @brief Cooperative scheduler that processes networking and mains sense changes.
 *
 * Must run frequently to keep MQTT responsive and ensure toggles are detected quickly. Each
 * iteration performs two jobs:
 *   1. Pump the networking layer so Wi-Fi/MQTT callbacks execute.
 *   2. Sample the mains sense input and translate toggles into brightness changes.
 */
void loop() {
  networkLoop();
  ensureOtaReady();
  if (otaReady && networkIsConnected()) {
    ArduinoOTA.handle();
  } else if (otaReady && !networkIsConnected()) {
    otaReady = false;
    networkSendSyslog("<134>dim-controller ota_stopped");
  }

  // Continuously read mains sense input to detect human gestures.
  const unsigned long now = millis();
  const int senseState = digitalRead(sensePin);

  if (senseState != lastSenseState) {
    const unsigned long delta = now - lastSenseChangeMs;
    if (delta < senseDebounceMs) {
      return;
    }
    const bool quickToggle = delta <= currentToggleWindowMs(); // below 50% we allow a longer window
    bool shouldReportHold = false; // defer logging until we know if we stayed in Hold
    lastSenseChangeMs = now;
    lastSenseState = senseState;
    bool brightnessChanged = false;

    // When mains drops and we were dim (<30%), force output off to avoid glow from slow discharge.
    if (senseState == LOW && rawToPercent(brightness) < 30) {
      forcedOffPreviousBrightness = brightness;
      applyBrightnessValue(0);
      forcedOffDueToLow = true;
      brightnessChanged = true;
    }

    if (quickToggle) {
      networkNotifyToggleDetected(senseState);
      handleQuickToggle(now);
      brightnessChanged = true;
    } else if (senseState == HIGH && forcedOffDueToLow) {
      // Restore previous brightness on a long-off/long-on cycle; treat as toggle only if quick.
      forcedOffDueToLow = false;
      if (delta <= currentToggleWindowMs()) {
        networkNotifyToggleDetected(senseState);
        handleQuickToggle(now);
        brightnessChanged = true;
      } else {
        applyBrightnessValue(forcedOffPreviousBrightness);
        persistBrightness();
        brightnessChanged = true;
      }
    } else {
      shouldReportHold = true;
    }

    // Push updated sense + brightness to MQTT so HA mirrors the physical change.
    if (!brightnessChanged) {
      networkNotifyBrightnessChange();
    }
    if (shouldReportHold) {
      reportStatus("hold_toggle");
    }
  }

  delay(5);
}
/** @brief Returns the quick-toggle window adjusted for current brightness. */
uint32_t currentToggleWindowMs() {
  return rawToPercent(brightness) < 50 ? lowBrightnessToggleWindowMs : toggleWindowMs;
}
