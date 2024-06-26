#pragma once

#include <Arduino.h>
#include <WrapperFreeRTOS.h>  // FreeRTOS C++ wrapper

enum class LogLevel { NONE, ERROR, WARN, INFO, DEBUG, VERBOSE };

void restartCPU(const char *TAG);

class LogEntry {
   public:
    LogEntry() = default;
    LogEntry(const char tag[11], const char message[51], LogLevel level);

    const char *getTag() { return m_tag; }
    const char *getMessage() { return m_message; }
    LogLevel getLogLevel() { return m_level; }

   private:
    char m_tag[11];
    char m_message[51];
    LogLevel m_level;
};

// This task will wait for a LogEntry to be received from the queue and then log it using the esp_log library.
class LogTask : public Task {
   public:
    LogTask(uint32_t stackSize = 2048, uint8_t priority = 1, uint8_t coreId = APP_CPU_NUM);

    void createQueue();
    void setup();
    void run(void *data);
    void queueLog(const char *tag, const char *message, LogLevel level);
    QueueHandle_t getQueueHandle();

   private:
    QueueHandle_t m_queueHandle;  // This queue is used to receive log messages from other tasks
};

extern LogTask logTask;
