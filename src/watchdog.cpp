#include "Watchdog.h"
#include <Arduino.h>
#include <map>
#include <string.h>

static uint32_t last_hw_kick = 0;
static std::map<String, uint32_t> task_last_ping;
#define SW_WDT_TIMEOUT_MS 500

void Watchdog_init() {
  last_hw_kick = millis();
  task_last_ping.clear();
}

void Watchdog_kickHardware() {
  // on ESP32 you could call esp_task_wdt_reset(); here we just record
  last_hw_kick = millis();
}

void Watchdog_kickTask(const char* name) {
  task_last_ping[String(name)] = millis();
}

bool Watchdog_check() {
  uint32_t now = millis();
  for (auto &kv : task_last_ping) {
    if ((int32_t)(now - kv.second) > SW_WDT_TIMEOUT_MS) {
      Serial.printf("[WDT] Task '%s' missed ping\n", kv.first.c_str());
      return false;
    }
  }
  return true;
}
