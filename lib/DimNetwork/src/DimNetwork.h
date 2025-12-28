/**
 * @file DimNetwork.h
 * @brief Wi-Fi + MQTT glue for the mains dimmer controller.
 *
 * Exposes a lightweight API the sketch can use to bootstrap connectivity while
 * the implementation handles state publishing and command parsing.
 */
#pragma once

#include <Arduino.h>

/** Operating mode for the dimmer core exposed to MQTT. */
enum class Mode : uint8_t { Hold, Fade };

/** Hardware polarity for the mains sense input. */
enum class SensePolarity : uint8_t { ActiveLow, ActiveHigh };
/** Adjust this constant if the sense optocoupler wiring inverts the signal. */
constexpr SensePolarity sensePolarity = SensePolarity::ActiveHigh;

/**
 * @brief Utility that maps a raw GPIO reading into a boolean mains-present flag.
 */
inline bool mainsPresentFromSense(int senseState) {
  return (sensePolarity == SensePolarity::ActiveLow) ? (senseState == LOW)
                                                     : (senseState != LOW);
}

/**
 * @brief Callbacks supplied by the main module so the network layer can query state.
 */
struct NetworkStatusView {
  /** Returns the current dimmer mode (Hold/Fade). */
  Mode (*getMode)();
  /** Returns the raw PWM brightness (0-1023). */
  uint16_t (*getBrightness)();
  /** Returns the latest mains sense GPIO reading. */
  int (*getSenseState)();
  /** Pretty-printer for the current mode used in logs. */
  const char *(*modeName)(Mode);
};

/**
 * @brief Initializes Wi-Fi + MQTT handling and begins connection attempts.
 *
 * @param ssid Wi-Fi SSID to join.
 * @param password Wi-Fi password.
 * @param connectTimeoutMs Time before a Wi-Fi attempt is considered failed.
 * @param retryIntervalMs Delay between subsequent Wi-Fi attempts.
 * @param statusView Callback bundle for pulling state.
 * @param holdAndPersistFn Function invoked when MQTT commands change brightness.
 * @param mqttHost MQTT broker hostname or IP.
 * @param mqttPort MQTT broker port.
 * @param mqttClientId Client identifier (optional; auto-generated when nullptr).
 * @param mqttUser Optional MQTT username.
 * @param mqttPassword Optional MQTT password.
 */
void networkInit(const char *ssid,
                 const char *password,
                 uint32_t connectTimeoutMs,
                 unsigned long retryIntervalMs,
                 const NetworkStatusView &statusView,
                 void (*holdAndPersistFn)(int value, const char *reason),
                 const char *mqttHost,
                 uint16_t mqttPort,
                 const char *mqttClientId,
                 const char *mqttUser = nullptr,
                 const char *mqttPassword = nullptr);

/** @brief Drives Wi-Fi + MQTT finite state machines; call frequently from loop(). */
void networkLoop();

/** @brief Notify the network layer that brightness changed so MQTT state can refresh. */
void networkNotifyBrightnessChange();

/** @brief Emit a syslog entry when a mains toggle gesture is detected. */
void networkNotifyToggleDetected(int senseState);

/** @brief Returns true when Wi-Fi is connected and has an IP. */
bool networkIsConnected();
/** @brief Returns true while the module is actively attempting to join Wi-Fi. */
bool networkIsConnecting();
/** @brief Returns true when the MQTT client is connected to the broker. */
bool networkIsMqttConnected();
