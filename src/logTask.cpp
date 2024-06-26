#include "logTask.hpp"

#include <Arduino.h>
#include <WrapperFreeRTOS.h>
#include <esp_log.h>

static const char *TAG = "LogTask";

LogTask::LogTask(uint32_t stackSize, uint8_t priority, uint8_t coreId)
    : Task("LogTask", stackSize, priority) {
    setCore(coreId);
    m_queueHandle = NULL;
}

void LogTask::createQueue() {
    m_queueHandle = xQueueCreate(10, sizeof(LogEntry));
    if (m_queueHandle == NULL) {
        ESP_LOGE(TAG, "Error creating the LogEntry queue");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ESP.restart();
    }
}

void LogTask::setup()
{
    createQueue();
}

// Run is the task's main function. It will be called once the task is started.
void LogTask::run(void *data) {
    LogEntry entry;
    while (true) {
        if (xQueueReceive(m_queueHandle, &entry, portMAX_DELAY)) {
            const char *tag = entry.getTag();
            const char *message = entry.getMessage();
            switch (entry.getLogLevel()) {
                case (LogLevel::ERROR):
                    ESP_LOGE(tag, "%s", message);
                    break;
                case (LogLevel::WARN):
                    ESP_LOGW(tag, "%s", message);
                    break;
                case (LogLevel::INFO):
                    ESP_LOGI(tag, "%s", message);
                    break;
                case (LogLevel::DEBUG):
                    ESP_LOGD(tag, "%s", message);
                    break;
                case (LogLevel::VERBOSE):
                    ESP_LOGV(tag, "%s", message);
                    break;
                default:
                    break;
            }
        }
    }
}

void LogTask::queueLog(const char *tag, const char *message, LogLevel level) {
    if ((int)level > CORE_DEBUG_LEVEL) {
        return;
    }
    LogEntry entry(tag, message, level);
    if (xQueueSend(m_queueHandle, &entry, portMAX_DELAY) != pdPASS) {
        ESP_LOGE(TAG, "queuePrint() - Error sending the message to the queue");
    }
}

QueueHandle_t LogTask::getQueueHandle() { return m_queueHandle; }

LogEntry::LogEntry(const char tag[11], const char message[51], LogLevel level) : m_level(level) {
    strcpy(m_tag, tag);
    strcpy(m_message, message);
}

// Global instance of the LogTask
LogTask logTask(2048, 1, PRO_CPU_NUM);

