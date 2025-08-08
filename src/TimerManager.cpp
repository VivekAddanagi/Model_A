#include "TimerManager.h"

TimerManager::TimerManager() : taskCount(0) {}

void TimerManager::addTask(TaskCallback cb, uint32_t intervalMs) {
    if (taskCount < MAX_TASKS) {
        tasks[taskCount] = { cb, intervalMs, millis() };
        taskCount++;
    }
}

void TimerManager::update() {
#if USE_HARDWARE_TIMER == 0
    uint32_t now = millis();
    for (int i = 0; i < taskCount; i++) {
        if (now - tasks[i].lastRun >= tasks[i].intervalMs) {
            tasks[i].lastRun = now;
            tasks[i].callback();
        }
    }
#endif
}

#if USE_HARDWARE_TIMER
void TimerManager::startHardwareTimer(uint32_t intervalMicros) {
    timer = timerBegin(0, 80, true); // 80 prescaler -> 1 tick = 1 microsecond
    timerAttachInterrupt(timer, &TimerManager::onTimerISR, true);
    timerAlarmWrite(timer, intervalMicros, true);
    timerAlarmEnable(timer);
}

void IRAM_ATTR TimerManager::onTimerISR() {
    // Future ISR-based task execution
    // Keep minimal: set flags or counters only
}
#endif
