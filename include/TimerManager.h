#ifndef TIMERMANAGER_H
#define TIMERMANAGER_H

#include <Arduino.h>
#include <functional>

#define USE_HARDWARE_TIMER 0   // 0 = millis() mode, 1 = ESP32 hardware timer

class TimerManager {
public:
    typedef std::function<void()> TaskCallback;

    struct Task {
        TaskCallback callback;
        uint32_t intervalMs;
        uint32_t lastRun;
    };

    TimerManager();

    // Add a new periodic task
    void addTask(TaskCallback cb, uint32_t intervalMs);

    // Call this inside loop() if in millis() mode
    void update();

    // Hardware timer setup (future use)
    void startHardwareTimer(uint32_t intervalMicros);

private:
    static const int MAX_TASKS = 10;
    Task tasks[MAX_TASKS];
    int taskCount;

#if USE_HARDWARE_TIMER
    hw_timer_t * timer = nullptr;
    static void IRAM_ATTR onTimerISR();
#endif
};

#endif
