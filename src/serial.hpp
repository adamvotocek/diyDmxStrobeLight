#pragma once

#include <Arduino.h>

class SerialLogTask {
   public:
    TaskHandle_t taskHandle;
    static QueueHandle_t queueHandle;

    SerialLogTask(const char *name, UBaseType_t priority, BaseType_t cpuCore);

    void initialize();
    static void serialTask(void *parameter);
    void queuePrint(const char *message);

   private:
    const char *name;
    UBaseType_t priority;
    BaseType_t cpuCore;
};