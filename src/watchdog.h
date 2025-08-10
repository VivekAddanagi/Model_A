#pragma once
#include <stdint.h>
void Watchdog_init();
void Watchdog_kickHardware();      // feed HW WDT (if present)
void Watchdog_kickTask(const char* name); // ping named task
bool Watchdog_check();  // returns true if OK
