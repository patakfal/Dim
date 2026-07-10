#include "SysLogger.h"

#include <Arduino.h>
#include <cstdarg>
#include <cstring>

namespace SysLogger {
namespace {
WiFiUDP udp;
IPAddress destination;
uint16_t destinationPort = 514;
String tag("esp8266");
String host("esp8266");
bool configured = false;

bool canSend() {
  return configured && WiFi.status() == WL_CONNECTED;
}

void sendPacket(const char *payload) {
  if (!payload || !canSend()) {
    return;
  }
  char packet[256];
  const int written = snprintf(packet,
                               sizeof(packet),
                               "<134>%s %s: %s",
                               host.c_str(),
                               tag.c_str(),
                               payload);
  if (written <= 0) {
    return;
  }
  const size_t length = static_cast<size_t>(written < static_cast<int>(sizeof(packet)) ? written
                                                                                      : static_cast<int>(sizeof(packet)) - 1);
  udp.beginPacket(destination, destinationPort);
  udp.write(reinterpret_cast<const uint8_t *>(packet), length);
  udp.endPacket();
}
}  // namespace

void begin(const IPAddress &serverAddr, uint16_t port, const char *appName, const char *hostName) {
  destination = serverAddr;
  destinationPort = port;
  udp.begin(0);  // bind an ephemeral port for outbound syslog
  if (appName && appName[0] != '\0') {
    tag = appName;
  } else {
    tag = "esp8266";
  }
  if (hostName && hostName[0] != '\0') {
    host = hostName;
  } else {
    host = WiFi.hostname();
    if (host.length() == 0) {
      host = "esp8266";
    }
  }
  configured = true;
}

void log(const char *message) {
  sendPacket(message);
}

void logf(const char *fmt, ...) {
  if (!fmt || !configured) {
    return;
  }
  char buffer[192];
  va_list args;
  va_start(args, fmt);
  int written = vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  if (written < 0) {
    return;
  }
  if (written >= static_cast<int>(sizeof(buffer))) {
    written = static_cast<int>(sizeof(buffer)) - 1;
    buffer[written] = '\0';
  }
  sendPacket(buffer);
}

}  // namespace SysLogger
