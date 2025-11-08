// Wi-Fi + MQTT integration. Keeps Home Assistant in sync with the dimmer and
// forwards incoming MQTT commands back to the main control module.
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "DimNetwork.h"

namespace {
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

const char *configuredSsid = nullptr;
const char *configuredPassword = nullptr;
uint32_t wifiConnectTimeoutMs = 0;
unsigned long wifiRetryIntervalMs = 0;

const char *mqttHost = nullptr;
uint16_t mqttPort = 1883;
const char *mqttClientId = nullptr;
const char *mqttUser = nullptr;
const char *mqttPassword = nullptr;

constexpr uint16_t pwmMaxValue = 1023;
constexpr uint16_t scalePercentToRaw(uint8_t percent) {
  return static_cast<uint16_t>((static_cast<uint32_t>(percent) * pwmMaxValue + 50U) / 100U);
}
constexpr uint8_t scaleRawToPercent(uint16_t raw) {
  return static_cast<uint8_t>((static_cast<uint32_t>(raw) * 100U + pwmMaxValue / 2U) / pwmMaxValue);
}

// Topic naming matches the Home Assistant MQTT light blueprint.
const char *topicState = "mydevice/light/state";
const char *topicSet = "mydevice/light/set";
const char *topicBrightness = "mydevice/light/brightness";
const char *topicBrightnessSet = "mydevice/light/brightness/set";
const char *topicAvailability = "mydevice/light/availability";

constexpr uint8_t mqttMinBrightnessPercent = 2;
constexpr uint16_t mqttMinBrightnessRaw = scalePercentToRaw(mqttMinBrightnessPercent);

NetworkStatusView statusView{};
void (*holdAndPersistFn)(int value, const char *reason) = nullptr;

bool wifiConnected = false;
bool wifiConnecting = false;
unsigned long wifiAttemptStartMs = 0;
unsigned long nextWifiAttemptMs = 0;
unsigned long lastMqttAttemptMs = 0;

String clientIdBuffer;

// Helpers ------------------------------------------------------------------

/**
 * @brief Returns true if the given string consists solely of digits.
 *
 * Used for validating MQTT payloads before attempting to convert to numbers.
 */
bool isNumeric(const String &text) {
  if (text.length() == 0) {
    return false;
  }
  for (size_t i = 0; i < text.length(); ++i) {
    if (!isDigit(text[i])) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Converts textual MQTT payloads (e.g. "42" or "42%") into PWM raw values.
 *
 * @return PWM raw value on success, or -1 when the payload is invalid.
 */
int parseBrightnessValue(String payload) {
  payload.trim();
  if (payload.length() == 0) {
    return -1;
  }
  if (payload.endsWith("%")) {
    payload.remove(payload.length() - 1);
  }
  payload.trim();
  if (!isNumeric(payload)) {
    return -1;
  }
  long value = payload.toInt();
  if (value < 0) {
    return -1;
  }
  if (value > 100) {
    value = 100;
  }
  return static_cast<int>(scalePercentToRaw(static_cast<uint8_t>(value)));
}

/**
 * @brief Clamps incoming MQTT brightness to the minimum safe glow.
 */
uint16_t clampMqttRaw(uint16_t raw) {
  if (raw < mqttMinBrightnessRaw) {
    return mqttMinBrightnessRaw;
  }
  return raw;
}

/**
 * @brief Publishes retained availability messages so Home Assistant knows the device is online.
 *
 * Uses a retained topic and caches the last state to avoid redundant publishes.
 */
void publishAvailability(bool online, bool force = false) {
  if (!mqttClient.connected()) {
    return;
  }
  static bool lastOnline = false;
  static bool hasAvailability = false;
  if (!force && hasAvailability && online == lastOnline) {
    return;
  }
  mqttClient.publish(topicAvailability, online ? "online" : "offline", true);
  lastOnline = online;
  hasAvailability = true;
}

/**
 * @brief Pushes light state + brightness to MQTT, caching desired brightness when mains are off.
 *
 * The cached value keeps Home Assistant in sync through mains glitches by reusing
 * the last commanded brightness once power returns.
 */
void publishStateAndBrightness(bool force = false) {
  if (!mqttClient.connected() || !statusView.getBrightness) {
    return;
  }
  static bool hasPublished = false;
  static uint16_t lastPublishedRaw = 0;
  static bool lastOn = false;
  static int lastSense = -1;
  static uint16_t lastDesiredRaw = 0;

  const uint16_t raw = statusView.getBrightness();
  const int sense = statusView.getSenseState
                        ? statusView.getSenseState()
                        : (sensePolarity == SensePolarity::ActiveLow ? HIGH : LOW);
  const bool mainsPresent = mainsPresentFromSense(sense);
  const bool isOn = mainsPresent && raw > 0;
  if (mainsPresent || raw > 0) {
    lastDesiredRaw = raw;
  }
  const uint16_t publishRaw = mainsPresent ? raw : lastDesiredRaw;

  if (!force && hasPublished && publishRaw == lastPublishedRaw && isOn == lastOn && sense == lastSense) {
    return;
  }

  mqttClient.publish(topicState, isOn ? "ON" : "OFF", true);
  const uint8_t scaled = scaleRawToPercent(publishRaw);
  char buffer[5];
  snprintf(buffer, sizeof(buffer), "%u", static_cast<unsigned>(scaled));
  mqttClient.publish(topicBrightness, buffer, true);

  lastPublishedRaw = publishRaw;
  lastOn = isOn;
  lastSense = sense;
  hasPublished = true;
}

/**
 * @brief MQTT callback that routes command topics to the dimmer core.
 *
 * Handles on/off requests as well as brightness changes, normalizing payloads
 * before invoking the holdAndPersist callback.
 */
void handleMqttMessage(char *topic, uint8_t *payload, unsigned int length) {
  String message;
  message.reserve(length);
  message.concat(reinterpret_cast<const char *>(payload), length);
  message.trim();

  if (strcmp(topic, topicSet) == 0) {
    String upper = message;
    upper.toUpperCase();
    if (upper == "ON") {
      if (holdAndPersistFn) {
        holdAndPersistFn(pwmMaxValue, "mqtt_state_on");
      }
    } else if (upper == "OFF") {
      if (holdAndPersistFn) {
        holdAndPersistFn(static_cast<int>(mqttMinBrightnessRaw), "mqtt_state_force_on");
      }
    } else {
      Serial.print(F("MQTT unknown state payload: "));
      Serial.println(message);
    }
    return;
  }

  if (strcmp(topic, topicBrightnessSet) == 0) {
    const int target = parseBrightnessValue(message);
    if (target >= 0) {
      if (holdAndPersistFn) {
        const uint16_t clamped = clampMqttRaw(static_cast<uint16_t>(target));
        holdAndPersistFn(static_cast<int>(clamped), "mqtt_brightness_set");
      }
    } else {
      Serial.print(F("MQTT invalid brightness payload: "));
      Serial.println(message);
    }
    return;
  }
}

/**
 * @brief Begins a Wi-Fi connection attempt if one is not already in progress.
 *
 * Resets timers so handleWiFiState() can track timeouts and retries.
 */
void startWiFiAttempt() {
  if (wifiConnecting || !configuredSsid) {
    return;
  }
  Serial.println(F("WiFi connecting"));
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(configuredSsid, configuredPassword);
  wifiConnecting = true;
  wifiConnected = false;
  wifiAttemptStartMs = millis();
  nextWifiAttemptMs = 0;
}

/**
 * @brief Cooperative Wi-Fi state machine that handles success, failure, and retry backoff.
 */
void handleWiFiState() {
  const unsigned long now = millis();
  const wl_status_t status = WiFi.status();

  if (wifiConnected) {
    if (status != WL_CONNECTED) {
      wifiConnected = false;
      wifiConnecting = false;
      WiFi.disconnect();
      nextWifiAttemptMs = now + wifiRetryIntervalMs;
      Serial.println(F("WiFi lost; will retry"));
      if (mqttClient.connected()) {
        mqttClient.disconnect();
      }
    }
    return;
  }

  if (wifiConnecting) {
    if (status == WL_CONNECTED) {
      wifiConnecting = false;
      wifiConnected = true;
      nextWifiAttemptMs = 0;
      Serial.print(F("WiFi connected IP="));
      Serial.println(WiFi.localIP());
      return;
    }

    if (status == WL_CONNECT_FAILED || status == WL_NO_SSID_AVAIL ||
        status == WL_CONNECTION_LOST || (now - wifiAttemptStartMs) >= wifiConnectTimeoutMs) {
      wifiConnecting = false;
      WiFi.disconnect();
      nextWifiAttemptMs = now + wifiRetryIntervalMs;
      Serial.println(F("WiFi connect failed; will retry"));
    }
    return;
  }

  if (nextWifiAttemptMs == 0 || now >= nextWifiAttemptMs) {
    startWiFiAttempt();
  }
}

/**
 * @brief Ensures the MQTT client is connected once Wi-Fi is available.
 *
 * Also refreshes the availability topic and configures the Last Will message.
 */
void ensureMqttConnection() {
  if (!wifiConnected || !mqttHost) {
    if (mqttClient.connected()) {
    publishAvailability(false, true);
    mqttClient.disconnect();
  }
  return;
  }

  if (mqttClient.connected()) {
    return;
  }

  const unsigned long now = millis();
  if (now - lastMqttAttemptMs < 2000UL) {
    return;
  }
  lastMqttAttemptMs = now;

  if (clientIdBuffer.isEmpty()) {
    if (mqttClientId && mqttClientId[0] != '\0') {
      clientIdBuffer = mqttClientId;
    } else {
      clientIdBuffer = F("dim-");
      clientIdBuffer += String(ESP.getChipId(), HEX);
    }
  }

  mqttClient.setServer(mqttHost, mqttPort);
  mqttClient.setCallback(handleMqttMessage);

  bool connected = false;
  if (mqttUser && mqttUser[0] != '\0') {
    connected = mqttClient.connect(clientIdBuffer.c_str(),
                                   mqttUser,
                                   mqttPassword,
                                   topicAvailability,
                                   0,
                                   true,
                                   "offline");
  } else {
    connected = mqttClient.connect(clientIdBuffer.c_str(),
                                   topicAvailability,
                                   0,
                                   true,
                                   "offline");
  }

  if (connected) {
    mqttClient.subscribe(topicSet);
    mqttClient.subscribe(topicBrightnessSet);
    publishAvailability(true, true);
    publishStateAndBrightness(true);
    Serial.println(F("MQTT connected"));
  } else {
    Serial.print(F("MQTT connect failed, rc="));
    Serial.println(mqttClient.state());
  }
}

}  // namespace

/**
 * @brief Initializes networking with credentials + callbacks supplied by the sketch.
 */
void networkInit(const char *ssid,
                 const char *password,
                 uint32_t connectTimeoutMs,
                 unsigned long retryIntervalMs,
                 const NetworkStatusView &view,
                 void (*holdAndPersist)(int value, const char *reason),
                 const char *mqttHostAddress,
                 uint16_t mqttHostPort,
                 const char *mqttClientIdentifier,
                 const char *mqttUserName,
                 const char *mqttUserPassword) {
  configuredSsid = ssid;
  configuredPassword = password;
  wifiConnectTimeoutMs = connectTimeoutMs;
  wifiRetryIntervalMs = retryIntervalMs;
  statusView = view;
  holdAndPersistFn = holdAndPersist;

  mqttHost = mqttHostAddress;
  mqttPort = mqttHostPort;
  mqttClientId = mqttClientIdentifier;
  mqttUser = mqttUserName;
  mqttPassword = mqttUserPassword;
  clientIdBuffer = "";
  lastMqttAttemptMs = 0;

  startWiFiAttempt();
}

/** @brief Pumps the Wi-Fi and MQTT state machines; call from the main loop. */
void networkLoop() {
  handleWiFiState();
  ensureMqttConnection();
  if (mqttClient.connected()) {
    mqttClient.loop();
  }
}

/** @brief Triggers an MQTT state publish when brightness changes. */
void networkNotifyBrightnessChange() {
  publishStateAndBrightness();
}

/** @brief Returns true when Wi-Fi is connected. */
bool networkIsConnected() {
  return wifiConnected;
}

/** @brief Returns true when a Wi-Fi attempt is currently running. */
bool networkIsConnecting() {
  return wifiConnecting;
}

/** @brief Returns true when the MQTT client is connected to the broker. */
bool networkIsMqttConnected() {
  return mqttClient.connected();
}
