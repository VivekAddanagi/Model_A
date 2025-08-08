#ifndef TIMERMANAGER_H
#define TIMERMANAGER_H

#include <Arduino.h>
#include <functional>

#define USE_HARDWARE_TIMER 0

class TimerManager {
public:
    typedef std::function<void()> TaskCallback;

    struct Task {
        TaskCallback callback = nullptr;
        uint32_t intervalMs = 0;
        uint32_t lastRun = 0;
        bool enabled = false;
    };

    TimerManager();

    // Add a periodic task. Returns true if added.
    bool addTask(TaskCallback cb, uint32_t intervalMs);

    // Call in loop() for millis()-based scheduling
    void update();

    // Stub for future hardware timer mode
    void startHardwareTimer(uint32_t intervalMicros);

private:
    static const int MAX_TASKS = 12;
    Task tasks[MAX_TASKS];
    int taskCount = 0;

#if USE_HARDWARE_TIMER
    hw_timer_t * timer = nullptr;
    static void IRAM_ATTR onTimerISR();
#endif
};

#endif // TIMERMANAGER_H
