#include "serial.hpp"

#include <Arduino.h>

QueueHandle_t SerialLogTask::queueHandle = NULL;

SerialLogTask::SerialLogTask(const char *name, UBaseType_t priority, BaseType_t cpuCore) {
    this->name = name;
    this->priority = priority;
    this->cpuCore = cpuCore;
    
    taskHandle = NULL;
    queueHandle = NULL;
}

void SerialLogTask::initialize()
{
    Serial.begin(115200);

    // Create queue for printing
    queueHandle = xQueueCreate(10, sizeof(char *));
    if (queueHandle == NULL) {
        Serial.println("Error creating the serial print queue");
    }
    // Create the task
    xTaskCreatePinnedToCore(serialTask, "SerialPrintTask", 1024, NULL, priority, &taskHandle, cpuCore);
}

void SerialLogTask::queuePrint(const char *string)
{
    char *message = (char *)malloc((strlen(string) + 1) * sizeof(char));  // Allocate memory for the string
    strcpy(message, string);  // Copy the string into the allocated memory
    if (&message == NULL) {
        Serial.println("Error allocating memory for the message");
        return;
    }
    if (xQueueSend(queueHandle, &message, portMAX_DELAY) != pdPASS) {
        Serial.println("Error sending the message to the queue");
    }
    free(message);
}

void SerialLogTask::serialTask(void *parameter) {
    char *message = NULL;
    while (true) {
        if (xQueueReceive(queueHandle, &message, portMAX_DELAY)) {
            Serial.println(message);
        }
    }
}
