#pragma once

#include <Arduino.h>
#include <WrapperFreeRTOS.h>  // FreeRTOS C++ wrapper

#include "dmxTask.hpp"

// Intervals for DMX channel 3 (light duration)
#define STROBE_BLACKOUT_MIN 0
#define STROBE_BLACKOUT_MAX 4
#define STROBE_DURATION_MIN 5
#define STROBE_DURATION_MAX 250
#define STROBE_LIGHT_FULL_ON_MIN 251
#define STROBE_LIGHT_FULL_ON_MAX 255

class strobeParameters {
   public:
    uint8_t dutyCycle;
    int lightOnTime;
    int lightOffTime;
    bool isLedOn;
    unsigned long lastSwitch;
};

class LedControlTask : public Task {
   public:
    LedControlTask(uint32_t stackSize = 2048, uint8_t priority = 1, uint8_t coreId = APP_CPU_NUM);

    void createTimers();
    void setup();
    void run(void *data);
    TimerHandle_t getLedOnTimerHandle() { return m_ledOnTimerHandle; };
    TimerHandle_t getLedOffTimerHandle() { return m_ledOffTimerHandle; };
    TimerHandle_t getSyncTimerHandle() { return m_syncTimerHandle; };

   private:
    static TimerHandle_t m_ledOnTimerHandle;
    static TimerHandle_t m_ledOffTimerHandle;
    static TimerHandle_t m_syncTimerHandle;
    static DmxMode<3> recievedDmx;
    static strobeParameters strobe;

    static void ledOnTimerCallback(TimerHandle_t xTimer);
    static void ledOffTimerCallback(TimerHandle_t xTimer);
    static void syncTimerCallback(TimerHandle_t xTimer);

    static int getStrobePeriod();
    static void setLightPwm();
    static void switchOnOff();
    static void refreshLedOnTimer();
    static void refreshLedOffTimer();
    static void refreshSyncTimer();
};

extern LedControlTask ledControlTask;