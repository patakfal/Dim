// Networking helpers for the dimmer controller. Sets up Wi-Fi, MQTT, and
// publishes state based on callbacks from the main module.
#pragma once

#include <Arduino.h>

enum class Mode : uint8_t { Hold, Fade };

enum class SensePolarity : uint8_t { ActiveLow, ActiveHigh };
// Update this constant if the mains sense signal is inverted in hardware.
constexpr SensePolarity sensePolarity = SensePolarity::ActiveHigh;

inline bool mainsPresentFromSense(int senseState) {
  return (sensePolarity == SensePolarity::ActiveLow) ? (senseState == LOW)
                                                     : (senseState != LOW);
}

// Callbacks supplied by the main module so the network layer can pull state.
struct NetworkStatusView {
  Mode (*getMode)();
  uint16_t (*getBrightness)();
  int (*getSenseState)();
  const char *(*modeName)(Mode);
};

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

void networkLoop();

void networkNotifyBrightnessChange();

bool networkIsConnected();
bool networkIsConnecting();
bool networkIsMqttConnected();

