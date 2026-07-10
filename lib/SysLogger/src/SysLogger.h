#pragma once

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

namespace SysLogger {

/**
 * @brief Initialize syslog destination and identifiers.
 *
 * @param serverAddr Syslog server IP.
 * @param port Syslog UDP port (default 514).
 * @param appName Tag that appears as the app name in syslog.
 * @param hostName Optional hostname to emit in syslog messages; falls back to Wi-Fi hostname.
 */
void begin(const IPAddress &serverAddr, uint16_t port, const char *appName, const char *hostName = nullptr);
void log(const char *message);
void logf(const char *fmt, ...);

}  // namespace SysLogger
