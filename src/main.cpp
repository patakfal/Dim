// Main application controlling ESP8266 based mains-sensing dimmer. Handles
// physical input timing, brightness persistence, and MQTT state reporting.
#include <Arduino.h>
#include <EEPROM.h>
#include <array>
#include <cstring>

#include "DimNetwork.h"

// Hardware wiring: D7 drives the triac gate (active low), D6 senses mains via opto.
const uint8_t ledPin = D7;
const uint8_t sensePin = D6;
constexpr uint16_t pwmMax = 1023;
const uint16_t fadeDelayMs = 500;
const uint32_t toggleWindowMs = 1000;

constexpr uint16_t percentToRaw(uint8_t percent) {
  return static_cast<uint16_t>((static_cast<uint32_t>(percent) * pwmMax + 50U) / 100U);
}

constexpr uint8_t rawToPercent(uint16_t raw) {
  return static_cast<uint8_t>((static_cast<uint32_t>(raw) * 100U + pwmMax / 2U) / pwmMax);
}

// Discrete brightness curve tuned for perceived linearity.
constexpr std::array<uint8_t, 20> brightnessStepPercents{
    2,  4,  6,  9,  12, 15, 18, 21, 24, 27,
    30, 33, 40, 47, 53, 60, 67, 80, 90, 100};

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
constexpr int fadeIndexStep = 1;

constexpr uint16_t minBrightnessRaw() {
  return brightnessStepRaws[0];
}

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

const char *mqttHost = "192.168.50.10"; // TODO: update to your MQTT broker
const uint16_t mqttPort = 1883;
const char *mqttClientId = "dim-controller";
const char *mqttUsername = "lynx";
const char *mqttPassword = "679ZMtgmzGWqknZ";

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

// Sense samples during boot jitter; take several readings and use majority vote.
// Ensures we don't misinterpret the mains state while GPIOs are still stabilizing.
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

// Convert a raw PWM value to the closest brightness step index for consistent fades.
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

// Apply raw PWM value while optionally tracking the closest step.
// This is the single writer to the LED output, so we keep all bookkeeping here.
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

// Clamp to legal step and write the corresponding raw value.
// Used by fades and MQTT commands that operate on discrete brightness steps.
void applyBrightnessIndex(int index) {
  index = constrain(index, 0, static_cast<int>(brightnessStepCount) - 1);
  brightnessIndex = index;
  const uint16_t stepped = brightnessStepRaws[index];
  applyBrightnessValue(stepped, false);
}

const char *modeName(Mode mode) {
  return mode == Mode::Fade ? "fade" : "hold";
}

// Writes a single-line status entry for serial debugging so we can trace mode transitions.
void reportStatus(const char *source) {
  const bool mainsPresent = mainsPresentFromSense(lastSenseState);
  const bool isOn = mainsPresent && brightness > 0;
  Serial.print(source);
  Serial.print(" mode=");
  Serial.print(modeName(currentMode));
  /*
  Serial.print(" sense_raw=");
  Serial.print(lastSenseState);
  Serial.print(" mains=");
  Serial.print(mainsPresent ? "ON" : "OFF");
  */
  Serial.print(" light=");
  Serial.print(isOn ? "ON" : "OFF");
  Serial.print(" brightness=");
  Serial.print(rawToPercent(brightness));
  Serial.println("%");
  // Serial.print("% pwm=");
  // Serial.println(pwmMax - brightness);
}

// Fetch brightness from EEPROM if signature matches; guarantees a sane fallback.
uint16_t loadBrightnessFromEEPROM() {
  if (EEPROM.read(eepromMagicAddr) == eepromMagic) {
    uint16_t stored = 0;
    EEPROM.get(eepromBrightnessAddr, stored);
    if (stored <= pwmMax) {
      lastSavedBrightness = stored;
      Serial.print(F("eeprom_load="));
      Serial.print(rawToPercent(stored));
      Serial.println('%');
      return stored;
    }
  }
  lastSavedBrightness = minBrightnessRaw();
  Serial.println(F("eeprom_load=default"));
  return lastSavedBrightness;
}

// Commit the current brightness to EEPROM if it changed, rate-limiting wear.
void persistBrightness() {
  const uint16_t value = brightness;
  if (value == lastSavedBrightness && EEPROM.read(eepromMagicAddr) == eepromMagic) {
    return; // nothing changed
  }

  EEPROM.write(eepromMagicAddr, eepromMagic);
  EEPROM.put(eepromBrightnessAddr, value);
  EEPROM.commit();
  lastSavedBrightness = value;
  Serial.print(F("eeprom_save="));
  Serial.print(rawToPercent(value));
  Serial.println('%');
}

// Switch into auto-fade mode, starting at the current step, and prime fade direction.
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

// Exit fade mode, store the resulting brightness, and rate-limit new fades.
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

// Prevent immediate re-entry into fade after recent state change.
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

// Set brightness, store it if needed, and publish status.
// All MQTT and manual hold actions funnel through here for consistent persistence.
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

Mode getCurrentMode() {
  return currentMode;
}

uint16_t getCurrentBrightness() {
  return brightness;
}

int getCurrentSenseState() {
  return lastSenseState;
}

// Interpret quick taps: double-tap to max, single to start fade.
// Uses a sliding window so mains glitches can't spam mode changes.
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

// Hardware + network initialization. Configures IO, loads brightness, wires callbacks.
void setup() {
  delay(200); // let it boot cleanly
  Serial.begin(115200);
  analogWriteFreq(1000);
  analogWriteRange(pwmMax);
  pinMode(ledPin, OUTPUT);
  pinMode(sensePin, INPUT);
  delay(10); // allow the sense line to settle before sampling
  EEPROM.begin(eepromSize);

  lastSenseState = readInitialSenseState();
  lastSenseChangeMs = millis();
  senseInactiveStartMs = mainsPresentFromSense(lastSenseState) ? 0 : lastSenseChangeMs;

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

// Cooperative scheduler: sample mains state, drive fades, and service MQTT/Wi-Fi.
void loop() {
  networkLoop();

  const unsigned long now = millis();
  const int senseState = digitalRead(sensePin);

  if (senseState != lastSenseState) {
    const unsigned long delta = now - lastSenseChangeMs;
    const bool quickToggle = delta <= toggleWindowMs;
    bool shouldReportHold = false;

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

    lastSenseChangeMs = now;
    lastSenseState = senseState;
    senseInactiveStartMs = mainsPresentFromSense(senseState) ? 0 : now;
    networkNotifyBrightnessChange();
    if (shouldReportHold) {
      reportStatus("hold_toggle");
    }
  } else if (!mainsPresentFromSense(senseState) && senseInactiveStartMs == 0) {
    senseInactiveStartMs = now;
  }

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
